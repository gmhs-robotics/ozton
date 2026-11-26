//! Canonical command composition and trajectory junction planning.
use std::time::{Duration, Instant};

use ozton_localization::LocalizationEstimate;
use ozton_math::{
    geometry::{DVec2, Pose, wrap_heading},
    path::{Limits, Path, MAX_POINTS},
    uom::si::f64::{Angle, Length},
};

use super::{Engine, MotionExecution, PathActions, PoseMotionError, TravelDirection};
use super::session::{Arrival, PathStage, PoseSession, Trajectory};
use crate::{MotionConfig, MotionResult, PathGuidance};

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
        Self { engine, command, stop: false, name: None, actions: PathActions::new() }
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
        self.engine.config.valid() && match self.command {
            Command::Path(path) => self.actions.valid(path),
            Command::Distance(d) => self.actions.is_empty() && d.value.is_finite(),
            Command::Heading(h) => self.actions.is_empty() && h.value.is_finite(),
            Command::Point(p) | Command::TurnPoint(p) => self.actions.is_empty() && p.is_finite(),
            Command::Pose(p) | Command::StraightPose(p) => self.actions.is_empty() && p.is_finite(),
            Command::Arc(r, s) => self.actions.is_empty() && r.value.is_finite()
                && r.value > 0.0 && s.value.is_finite() && s.value.abs() <= core::f64::consts::TAU,
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
        Self { segments: Vec::new() }
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
enum Junction { Blend, Reverse, Stop }

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

fn restrictive(mut a: Limits, b: Limits) -> Limits {
    a.linear_speed = a.linear_speed.min(b.linear_speed);
    a.wheel_speed = a.wheel_speed.min(b.wheel_speed);
    a.acceleration = a.acceleration.min(b.acceleration);
    a.deceleration = a.deceleration.min(b.deceleration);
    a.lateral_acceleration = a.lateral_acceleration.min(b.lateral_acceleration);
    a.wheel_acceleration = a.wheel_acceleration.min(b.wheel_acceleration);
    a
}

fn scale(config: MotionConfig) -> f64 {
    if config.path.guidance == PathGuidance::PurePursuit { 1.0 }
    else { config.path.reference_speed_scale }
}

async fn execute<D: MotionExecution>(
    drive: &mut D,
    segments: &mut [MotionSegment<'_>],
) -> MotionResult<D> {
    if segments.iter().any(|s| !s.valid()) {
        return Err(PoseMotionError::Configuration);
    }
    let mut index = 0;
    while index < segments.len() {
        if !drive.motion_active() { return Err(PoseMotionError::Cancelled); }
        if let Some(path) = segments[index].path() {
            let mut end = index + 1;
            let mut count = path.samples().len();
            while end < segments.len() && junction(&segments[end - 1], &segments[end]) == Junction::Blend {
                count += segments[end].path().unwrap().samples().len() - 1;
                if count > MAX_POINTS {
                    // A separate stopped group is preferable to exceeding the bounded path storage.
                    break;
                }
                end += 1;
            }
            let arrival = if end < segments.len() && junction(&segments[end - 1], &segments[end]) == Junction::Reverse {
                Arrival::Reverse
            } else { Arrival::Stop };
            run_paths(drive, &mut segments[index..end], arrival).await?;
            index = end;
        } else {
            segments[index].direct(drive).await?;
            index += 1;
        }
    }
    Ok(())
}

async fn run_paths<D: MotionExecution>(
    drive: &mut D,
    segments: &mut [MotionSegment<'_>],
    arrival: Arrival,
) -> MotionResult<D> {
    let paths: Vec<_> = segments.iter().map(|s| s.path().unwrap()).collect();
    let reverse = segments[0].reverse();
    let first_config = segments[0].engine.config;
    let first_scale = scale(first_config);
    let mut limits = paths[0].limits();
    let mut minimum_scale = first_scale;
    let mut uniform_scale = true;
    for (segment, path) in segments.iter().zip(&paths) {
        limits = restrictive(limits, path.limits());
        limits = restrictive(limits, segment.engine.config.path_limits(limits.track_width));
        let s = scale(segment.engine.config);
        minimum_scale = minimum_scale.min(s);
        uniform_scale &= (s - first_scale).abs() < 1e-9;
    }
    // A mixed time scale must not produce a reference-velocity discontinuity.
    // Use the conservative common speed ceiling, not independent time dilation.
    if !uniform_scale {
        limits.linear_speed.value *= minimum_scale;
        limits.wheel_speed.value *= minimum_scale;
    }
    let joined = if segments.len() > 1 {
        Some(Path::join_cooperative(&paths, limits, async || {
            drive.wait_until(Instant::now()).await;
            drive.motion_active()
        }).await.map_err(|e| {
            if e == ozton_math::path::PathError::Cancelled { PoseMotionError::Cancelled }
            else { PoseMotionError::Configuration }
        })?)
    } else { None };
    let path = joined.as_ref().unwrap_or(paths[0]);
    let start = body_pose(path.start().pose, reverse);
    let target = body_pose(path.end().pose, reverse);
    let actual = drive.localization().pose();
    if !actual.is_finite() { return Err(PoseMotionError::Configuration); }
    if actual.position.distance(start.position) > 0.08
        || wrap_heading(start.heading - actual.heading).value.abs() > 0.35
    {
        ozton_diagnostics::log_event!(Warn, "motion", "Trajectory start offset exceeds tuning threshold; continuing feedback");
    }
    let options = segments[0].engine.options(drive).ok_or(PoseMotionError::Configuration)?;
    let mut distance = 0.0;
    let mut timeout = Duration::ZERO;
    let mut stages = Vec::with_capacity(segments.len());
    for (segment, source) in segments.iter_mut().zip(paths) {
        distance += source.end().distance;
        let mut config = segment.engine.config;
        if !uniform_scale { config.path.reference_speed_scale = 1.0; }
        timeout = timeout.saturating_add(config.timing.stage_timeout);
        stages.push(PathStage { config, end_distance: distance.min(path.end().distance),
            name: segment.name, actions: &mut segment.actions });
    }
    stages.last_mut().unwrap().end_distance = path.end().distance;
    drive.localization_mut().release_wall_contact();
    let mut session = PoseSession::new(drive, options, stages[0].config)?;
    let result = session.follow_group(Trajectory { path, reverse, target, timeout }, &mut stages, arrival).await;
    session.finish(result)
}

#[cfg(all(test, not(target_os = "vexos")))]
mod tests {
    use super::*;
    use crate::{PurePursuit, Ramsete};
    use ozton_math::{path::Waypoint, qty};
    fn path(a: f64, b: f64) -> Path {
        Path::new(&[Waypoint::new(DVec2::new(0.0, a), 1.0),
                    Waypoint::new(DVec2::new(0.0, b), 1.0)],
            Limits::from_units(qty!(0.33 m), qty!(1 m/s), qty!(1 m/s),
                qty!(1 m/s^2), qty!(1 m/s^2), qty!(1 m/s^2))).unwrap()
    }

    #[test]
    fn junctions_infer_blends_reversals_and_explicit_stops() {
        let a = path(0.0, 1.0);
        let b = path(1.0, 2.0);
        let c = path(1.0, 0.0);
        let pp = PurePursuit::new().follow(&a);
        assert_eq!(junction(&pp, &Ramsete::new().follow(&b)), Junction::Blend);
        assert_eq!(junction(&pp, &Ramsete::new().with_travel(TravelDirection::Backward).follow(&c)), Junction::Reverse);
        assert_eq!(junction(&pp, &PurePursuit::new().follow(&c)), Junction::Stop);
        let chain = MotionChain::new().then_stop(pp).then(Ramsete::new().follow(&b));
        assert_eq!(junction(&chain.segments[0], &chain.segments[1]), Junction::Stop);
    }
}
