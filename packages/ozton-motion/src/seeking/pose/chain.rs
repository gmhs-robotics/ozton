//! Canonical command composition and trajectory junction planning.
use std::time::{Duration, Instant};

use ozton_localization::LocalizationEstimate;
use ozton_math::{
    geometry::{DVec2, Pose, wrap_heading},
    path::{Limits, MAX_POINTS, Path},
    uom::si::f64::{Angle, Length},
};

use super::{
    Engine, MotionExecution, PathActions, PoseMotionError, TravelDirection,
    session::{Arrival, PathStage, PoseSession, Trajectory},
};
use crate::{MotionConfig, MotionResult, config::blended_limits};

#[derive(Clone, Copy)]
pub(in crate::seeking) enum Command<'a> {
    Path(&'a Path),
    Distance(Length),
    Heading(Angle),
    TurnPoint(DVec2),
    Point(DVec2),
    StraightPose(Pose),
    Pose(Pose),
    Arc(Length, Angle),
}

/// A bound controller command; usable alone or as one step in MotionChain.
pub struct MotionSegment<'a> {
    engine: Engine,
    command: Command<'a>,
    stop: bool,
    name: Option<&'a str>,
    actions: PathActions<'a>,
}

impl<'a> MotionSegment<'a> {
    pub(in crate::seeking) fn new(engine: Engine, command: Command<'a>) -> Self {
        Self {
            engine,
            command,
            stop: false,
            name: None,
            actions: PathActions::new(),
        }
    }

    #[must_use]
    pub fn named(mut self, name: &'a str) -> Self {
        self.name = Some(name);
        self
    }

    /// Attach one-shot mechanism actions to a path segment.
    #[must_use]
    pub fn with_actions(mut self, actions: PathActions<'a>) -> Self {
        self.actions = actions;
        self
    }

    pub async fn run<D: MotionExecution>(self, drive: &mut D) -> MotionResult<D> {
        RunMotion::run(self, drive).await
    }

    fn reverse(&self) -> bool {
        self.engine.travel == Some(TravelDirection::Backward)
    }

    fn path(&self) -> Option<&'a Path> {
        match self.command {
            Command::Path(path) => Some(path),
            _ => None,
        }
    }

    fn valid(&self) -> bool {
        self.engine.config.valid()
            && match self.command {
                Command::Path(path) => self.actions.valid(path),
                Command::Distance(d) => self.actions.is_empty() && d.value.is_finite(),
                Command::Heading(h) => self.actions.is_empty() && h.value.is_finite(),
                Command::Point(p) | Command::TurnPoint(p) => {
                    self.actions.is_empty() && p.is_finite()
                }
                Command::Pose(p) | Command::StraightPose(p) => {
                    self.actions.is_empty() && p.is_finite()
                }
                Command::Arc(r, s) => {
                    self.actions.is_empty()
                        && r.value.is_finite()
                        && r.value > 0.0
                        && s.value.is_finite()
                        && s.value.abs() <= core::f64::consts::TAU
                }
            }
    }

    async fn direct<D: MotionExecution>(&self, drive: &mut D) -> MotionResult<D> {
        match self.command {
            Command::Path(_) => unreachable!("path uses shared trajectory executor"),
            Command::Distance(d) => self.engine.drive_distance(drive, d).await,
            Command::Heading(h) => self.engine.turn_to_heading(drive, h).await,
            Command::TurnPoint(p) => self.engine.turn_to_point(drive, p).await,
            Command::Point(p) => self.engine.move_to_point(drive, p).await,
            Command::StraightPose(p) => self.engine.move_to_pose_straight(drive, p).await,
            Command::Pose(p) => self.engine.move_to_pose(drive, p).await,
            Command::Arc(r, s) => self.engine.drive_arc(drive, r, s).await,
        }
    }
}

/// The common execution trait for any bound command or composed chain.
#[allow(async_fn_in_trait)]
pub trait RunMotion {
    async fn run<D: MotionExecution>(self, drive: &mut D) -> MotionResult<D>;
}

impl RunMotion for MotionSegment<'_> {
    async fn run<D: MotionExecution>(self, drive: &mut D) -> MotionResult<D> {
        execute(drive, &mut [self]).await
    }
}

/// Compose commands with automatic compatible path blending and reversal cusps.
#[derive(Default)]
pub struct MotionChain<'a> {
    segments: Vec<MotionSegment<'a>>,
}

impl<'a> MotionChain<'a> {
    pub const fn new() -> Self {
        Self {
            segments: Vec::new(),
        }
    }

    #[must_use]
    pub fn then(mut self, segment: MotionSegment<'a>) -> Self {
        self.segments.push(segment);
        self
    }

    /// Require full settling at this segment's end even if the next can blend.
    #[must_use]
    pub fn then_stop(mut self, mut segment: MotionSegment<'a>) -> Self {
        segment.stop = true;
        self.segments.push(segment);
        self
    }

    pub async fn run<D: MotionExecution>(self, drive: &mut D) -> MotionResult<D> {
        RunMotion::run(self, drive).await
    }
}

impl RunMotion for MotionChain<'_> {
    async fn run<D: MotionExecution>(mut self, drive: &mut D) -> MotionResult<D> {
        execute(drive, &mut self.segments).await
    }
}

fn body_pose(mut pose: Pose, reverse: bool) -> Pose {
    if reverse {
        pose.heading += ozton_math::qty!(180 deg);
    }
    pose
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Junction {
    Blend,
    Reverse,
    Stop,
}

fn junction(a: &MotionSegment<'_>, b: &MotionSegment<'_>) -> Junction {
    if a.stop {
        return Junction::Stop;
    }
    let (Some(p), Some(q)) = (a.path(), b.path()) else {
        return Junction::Stop;
    };
    let end = body_pose(p.end().pose, a.reverse());
    let start = body_pose(q.start().pose, b.reverse());
    if end.position.distance(start.position) > 1e-6
        || wrap_heading(end.heading - start.heading).value.abs() > 0.04
    {
        return Junction::Stop;
    }
    if a.reverse() != b.reverse() {
        return Junction::Reverse;
    }
    if a.engine.config.timing.period != b.engine.config.timing.period
        || a.engine.config.timing.maximum_gap != b.engine.config.timing.maximum_gap
        || (p.limits().track_width.value - q.limits().track_width.value).abs() > 1e-8
    {
        return Junction::Stop;
    }
    Junction::Blend
}

struct PathPlan {
    start: usize,
    end: usize,
    arrival: Arrival,
    joined: Option<Path>,
    uniform_scale: bool,
}

enum PlannedMotion {
    Direct(usize),
    Path(PathPlan),
}

async fn execute<D: MotionExecution>(
    drive: &mut D,
    segments: &mut [MotionSegment<'_>],
) -> MotionResult<D> {
    if segments.iter().any(|s| {
        !s.valid()
            || s.path().is_some_and(|p| {
                (p.limits().track_width.value - drive.track_width().value).abs() > 1e-8
            })
    }) {
        return Err(PoseMotionError::Configuration);
    }
    // Plan all compatible groups before output starts. No path joining or
    // allocation-heavy reparameterization at a physical reversal.
    let mut plan = Vec::new();
    let mut index = 0;
    while index < segments.len() {
        if !drive.motion_active() {
            return Err(PoseMotionError::Cancelled);
        }
        if let Some(path) = segments[index].path() {
            let mut end = index + 1;
            let mut count = path.samples().len();
            while end < segments.len()
                && junction(&segments[end - 1], &segments[end]) == Junction::Blend
            {
                count += segments[end].path().unwrap().samples().len() - 1;
                if count > MAX_POINTS {
                    break;
                }
                end += 1;
            }
            let arrival = if end < segments.len()
                && junction(&segments[end - 1], &segments[end]) == Junction::Reverse
            {
                Arrival::Reverse
            } else {
                Arrival::Stop
            };
            let (joined, uniform_scale) = plan_paths(drive, &segments[index..end]).await?;
            plan.push(PlannedMotion::Path(PathPlan {
                start: index,
                end,
                arrival,
                joined,
                uniform_scale,
            }));
            index = end;
        } else {
            plan.push(PlannedMotion::Direct(index));
            index += 1;
        }
    }
    for step in plan {
        if !drive.motion_active() {
            return Err(PoseMotionError::Cancelled);
        }
        match step {
            PlannedMotion::Path(group) => {
                run_paths(drive, &mut segments[group.start..group.end], &group).await?;
            }
            PlannedMotion::Direct(index) => {
                let segment = &segments[index];
                segment.direct(drive).await?;
                if let Some(name) = segment.name {
                    let pose = drive.localization().pose();
                    ozton_diagnostics::log_row!(
                        Info,
                        "MOTION_DIRECT_RESULT",
                        "stage={name},x_m={:.6},y_m={:.6},heading_rad={:.6}",
                        pose.position.x,
                        pose.position.y,
                        pose.heading.value
                    );
                    let _ = (name, pose);
                }
            }
        }
    }
    Ok(())
}

async fn plan_paths<D: MotionExecution>(
    drive: &mut D,
    segments: &[MotionSegment<'_>],
) -> MotionResult<D, (Option<Path>, bool)> {
    if segments.len() == 1 {
        return Ok((None, true));
    }
    drive.brake().map_err(PoseMotionError::StopFailed)?;
    let paths: Vec<_> = segments.iter().map(|s| s.path().unwrap()).collect();
    let (limits, uniform_scale) =
        blended_limits(segments.iter().map(|s| (s.path().unwrap(), s.engine.config)))
            .ok_or(PoseMotionError::Configuration)?;
    let joined = if segments.len() > 1 {
        Some(
            Path::join_cooperative(&paths, limits, async || {
                drive
                    .wait_until(Instant::now() + Duration::from_millis(1))
                    .await;
                drive.motion_active()
            })
            .await
            .map_err(|e| {
                if e == ozton_math::path::PathError::Cancelled {
                    PoseMotionError::Cancelled
                } else {
                    PoseMotionError::Configuration
                }
            })?,
        )
    } else {
        None
    };
    Ok((joined, uniform_scale))
}

async fn run_paths<D: MotionExecution>(
    drive: &mut D,
    segments: &mut [MotionSegment<'_>],
    plan: &PathPlan,
) -> MotionResult<D> {
    let reverse = segments[0].reverse();
    let path = plan.joined.as_ref().unwrap_or(segments[0].path().unwrap());
    let start = body_pose(path.start().pose, reverse);
    let target = body_pose(path.end().pose, reverse);
    let actual = drive.localization().pose();
    if !actual.is_finite() {
        return Err(PoseMotionError::Configuration);
    }
    if actual.position.distance(start.position) > 0.08
        || wrap_heading(start.heading - actual.heading).value.abs() > 0.35
    {
        ozton_diagnostics::log_event!(
            Warn,
            "motion",
            "Trajectory start offset exceeds tuning threshold; continuing feedback"
        );
    }
    let options = segments[0]
        .engine
        .options(drive)
        .ok_or(PoseMotionError::Configuration)?;
    let mut distance = 0.0;
    let mut timeout = Duration::ZERO;
    let mut stages = Vec::with_capacity(segments.len());
    for segment in segments.iter_mut() {
        distance += segment.path().unwrap().end().distance;
        let mut config = segment.engine.config;
        if !plan.uniform_scale {
            config.path.reference_speed_scale = 1.0;
        }
        timeout = timeout.saturating_add(config.timing.stage_timeout);
        stages.push(PathStage {
            config,
            end_distance: distance.min(path.end().distance),
            name: segment.name,
            actions: &mut segment.actions,
        });
    }
    stages.last_mut().unwrap().end_distance = path.end().distance;
    drive.localization_mut().release_wall_contact();
    let mut session = PoseSession::new(drive, options, stages[0].config)?;
    let result = session
        .follow_group(
            Trajectory {
                path,
                reverse,
                target,
                timeout,
            },
            &mut stages,
            plan.arrival,
        )
        .await;
    session.finish(result)
}

#[cfg(all(test, not(target_os = "vexos")))]
mod tests {
    use ozton_math::{path::Waypoint, qty};

    use super::*;
    use crate::{PurePursuit, Ramsete};
    fn path(a: f64, b: f64) -> Path {
        Path::new(
            &[
                Waypoint::new(DVec2::new(0.0, a), 1.0),
                Waypoint::new(DVec2::new(0.0, b), 1.0),
            ],
            Limits::from_units(
                qty!(0.33 m),
                qty!(1 m/s),
                qty!(1 m/s),
                qty!(1 m/s^2),
                qty!(1 m/s^2),
                qty!(1 m/s^2),
            ),
        )
        .unwrap()
    }

    #[test]
    fn junctions_infer_blends_reversals_and_explicit_stops() {
        let a = path(0.0, 1.0);
        let b = path(1.0, 2.0);
        let c = path(1.0, 0.0);
        let pp = PurePursuit::new().follow(&a);
        assert_eq!(junction(&pp, &Ramsete::new().follow(&b)), Junction::Blend);
        assert_eq!(
            junction(
                &pp,
                &Ramsete::new()
                    .with_travel(TravelDirection::Backward)
                    .follow(&c)
            ),
            Junction::Reverse
        );
        assert_eq!(
            junction(&pp, &PurePursuit::new().follow(&c)),
            Junction::Stop
        );
        let chain = MotionChain::new()
            .then_stop(pp)
            .then(Ramsete::new().follow(&b));
        assert_eq!(
            junction(&chain.segments[0], &chain.segments[1]),
            Junction::Stop
        );
    }

    /// Ideal differential plant driven through the real asynchronous executor.
    /// Use virtual feedback time and a bounded real scheduling tick.
    mod execution {
        use std::{
            cell::Cell,
            future::{Future, ready},
            task::{Context, Poll, Waker},
        };

        use ozton_drivetrain::{HasLocalization, model::DriveEncoders};
        use ozton_localization::{Fault, Feedback};
        use ozton_math::{
            geometry::BodyDelta,
            path::clearance::{Footprint, WallContact},
            uom::{ConstZero, si::f64::Velocity},
        };

        use super::*;

        struct Estimate(Feedback);
        impl LocalizationEstimate for Estimate {
            fn pose(&self) -> Pose {
                self.0.pose
            }
            fn pose_trusted(&self) -> bool {
                true
            }
            fn feedback(&self) -> Result<Option<Feedback>, Fault> {
                Ok(Some(self.0))
            }
            fn wall_contact(&self) -> Option<WallContact> {
                None
            }
            fn release_wall_contact(&mut self) {}
            fn request_distance_reset(&mut self) {}
            fn cancel_distance_reset(&mut self) {}
        }

        struct Plant {
            estimate: Estimate,
            wheels: [Velocity; 2],
            commands: Vec<(f64, f64)>,
            stops: Vec<f64>,
        }

        impl Plant {
            fn new() -> Self {
                Self {
                    estimate: Estimate(Feedback {
                        time: Duration::ZERO,
                        pose: Pose::default(),
                        forward_travel: Length::ZERO,
                        forward_velocity: Velocity::ZERO,
                        lateral_velocity: Velocity::ZERO,
                        angular_velocity: qty!(0 rad/s),
                        ranges: [None; _],
                        reset_accepted: false,
                    }),
                    wheels: [Velocity::ZERO; 2],
                    commands: Vec::new(),
                    stops: Vec::new(),
                }
            }
        }

        impl HasLocalization for Plant {
            type Localization = Estimate;
            fn localization(&self) -> impl std::ops::Deref<Target = Estimate> {
                &self.estimate
            }
            fn localization_mut(&mut self) -> impl std::ops::DerefMut<Target = Estimate> {
                &mut self.estimate
            }
        }
        impl DriveEncoders for Plant {
            fn track_width(&self) -> Length {
                qty!(0.33 m)
            }
            fn tracking_positions(&mut self) -> [Option<Length>; 2] {
                [None; 2]
            }
        }
        impl MotionExecution for Plant {
            type Error = std::convert::Infallible;
            fn footprint(&self) -> Option<Footprint> {
                None
            }
            fn drive_wheels(&mut self, speeds: [Velocity; 2]) -> Result<(), Self::Error> {
                self.wheels = speeds;
                self.commands.push((
                    self.estimate.0.pose.position.y,
                    (speeds[0].value + speeds[1].value) * 0.5,
                ));
                Ok(())
            }
            fn brake(&mut self) -> Result<(), Self::Error> {
                self.stops.push(self.estimate.0.pose.position.y);
                self.wheels = [Velocity::ZERO; 2];
                Ok(())
            }
            fn emergency_brake(&mut self) -> Result<(), Self::Error> {
                self.brake()
            }
            fn wait_until(&mut self, _deadline: Instant) -> impl Future<Output = ()> {
                std::thread::sleep(Duration::from_micros(100));
                let dt = 0.01;
                let v = (self.wheels[0].value + self.wheels[1].value) * 0.5;
                let w = (self.wheels[1].value - self.wheels[0].value) / self.track_width().value;
                let f = &mut self.estimate.0;
                f.time += Duration::from_millis(10);
                f.pose = f.pose.integrate(BodyDelta {
                    translation: DVec2::new(0.0, v * dt),
                    heading: qty!((w * dt) rad),
                });
                f.forward_travel += qty!((v * dt) m);
                f.forward_velocity = qty!((v) m/s);
                f.angular_velocity = qty!((w) rad/s);
                ready(())
            }
            fn motion_active(&self) -> bool {
                self.estimate.0.time < Duration::from_secs(20)
            }
        }

        fn dense_path(a: f64, b: f64, limits: Limits) -> Path {
            let points: Vec<_> = (0..=20)
                .map(|i| Waypoint::new(DVec2::new(0.0, a + (b - a) * i as f64 / 20.0), 1.0))
                .collect();
            Path::new(&points, limits).unwrap()
        }

        fn run<F: Future>(future: F) -> F::Output {
            let mut future = std::pin::pin!(future);
            let mut context = Context::from_waker(Waker::noop());
            match future.as_mut().poll(&mut context) {
                Poll::Ready(value) => value,
                Poll::Pending => panic!("ideal plant suspended unexpectedly"),
            }
        }

        #[test]
        fn live_chain_preserves_speed_across_same_and_mixed_guidance_and_reverses() {
            let config = MotionConfig::new().with_limits(
                crate::MotionLimits::new()
                    .with_wheel_speed(qty!(0.8 m/s))
                    .with_acceleration(qty!(1 m/s^2))
                    .with_deceleration(qty!(1 m/s^2)),
            );
            let limits = config.path_limits(qty!(0.33 m));
            let a = dense_path(0.0, 0.5, limits);
            let b = dense_path(0.5, 1.0, limits);
            let c = dense_path(1.0, 1.5, limits);
            let d = dense_path(1.5, 0.8, limits);
            let mut plant = Plant::new();
            let events = Cell::new(0);
            let pursuit = PurePursuit::new().with_config(config);
            let arrival = || {
                events.set(events.get() + 1);
                Ok::<_, std::convert::Infallible>(())
            };
            run(MotionChain::new()
                .then(
                    pursuit
                        .follow(&a)
                        .with_actions(PathActions::new().on_arrival(arrival)),
                )
                .then(
                    pursuit
                        .follow(&b)
                        .with_actions(PathActions::new().on_arrival(arrival)),
                )
                .then(Ramsete::new().with_config(config).follow(&c))
                .then(pursuit.with_travel(TravelDirection::Backward).follow(&d))
                .run(&mut plant))
            .unwrap();
            assert_eq!(events.get(), 2);
            for seam in [0.5, 1.0] {
                assert!(
                    plant
                        .commands
                        .iter()
                        .any(|&(y, v)| (y - seam).abs() < 0.04 && v > 0.3)
                );
                assert!(
                    !plant.stops.iter().any(|&y| (y - seam).abs() < 0.04),
                    "unexpected stop near {seam}: {:?}",
                    plant.stops
                );
            }
            assert!(plant.stops.iter().any(|&y| (y - 1.5).abs() < 0.04));
            assert!(plant.commands.iter().any(|&(y, v)| y > 1.2 && v < -0.1));
            assert!((plant.estimate.0.pose.position.y - 0.8).abs() < 0.04);
            assert!(plant.estimate.0.pose.heading.value.abs() < 0.06);
        }
    }
}
