use std::time::{Duration, Instant};

use ozton_localization::Feedback;
use ozton_math::{
    geometry::wrap_heading,
    uom::si::{f64::Velocity, velocity::meter_per_second},
};
use ozton_runtime::{clock::CommandClock, competition::autonomous_active, log_row};

use super::{PoseDrive, PoseMotionError};
use crate::basic::differential::{DifferentialFeedback, DifferentialTarget, DriveOptions};

pub(super) struct PoseSession<'a, D: PoseDrive> {
    pub(super) drive: &'a mut D,
    pub(super) options: DriveOptions,
    config: crate::MotionConfig,
    clock: CommandClock,
    started: Instant,
    finished: bool,
}

impl<'a, D: PoseDrive> PoseSession<'a, D> {
    pub(super) fn new(
        drive: &'a mut D,
        options: DriveOptions,
        config: crate::MotionConfig,
    ) -> Result<Self, PoseMotionError<D::Error>> {
        if !options.valid() || !config.valid() {
            return Err(PoseMotionError::Configuration);
        }
        Ok(Self {
            drive,
            options,
            config,
            clock: CommandClock::new(config.period).ok_or(PoseMotionError::Configuration)?,
            started: Instant::now(),
            finished: false,
        })
    }

    async fn tick(&mut self) -> Result<Duration, PoseMotionError<D::Error>> {
        let dt = self.clock.tick().await;
        if !autonomous_active() {
            return Err(PoseMotionError::Cancelled);
        }
        if dt.is_zero() || dt > self.config.maximum_gap {
            return Err(PoseMotionError::ControlGap);
        }
        Ok(dt)
    }

    pub(super) async fn run(
        &mut self,
        target: DifferentialTarget,
        timeout: Duration,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let status = match target {
            DifferentialTarget::Turn { .. } | DifferentialTarget::TurnToPoint { .. } => {
                "Autonomous: turn"
            }
            DifferentialTarget::DriveToPoint { .. } => "Autonomous: drive to point",
            DifferentialTarget::DriveDistance { .. } => "Autonomous: drive distance",
            DifferentialTarget::DriveArc { .. } => "Autonomous: drive arc",
        };
        log_row!(
            Debug,
            "MOTION_COMMAND",
            "state=start,command={status},timeout_ms={}",
            timeout.as_millis()
        );
        let result = self.run_stage(target, timeout, status).await;
        match &result {
            Ok(()) => log_row!(Debug, "MOTION_COMMAND", "state=complete,command={status}"),
            Err(error) => log_row!(
                Debug,
                "MOTION_COMMAND",
                "state=failed,command={status},reason={}",
                error.reason()
            ),
        }
        result
    }

    async fn run_stage(
        &mut self,
        target: DifferentialTarget,
        timeout: Duration,
        status: &str,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let started = Instant::now();
        let mut controller = DifferentialFeedback::new(self.config.tuning).with_config(self.config);
        let mut settled = Duration::ZERO;
        let mut progress = Progress::default();
        let mut last_feedback = None;
        let mut feedback_at = Instant::now();
        loop {
            let dt = self.tick().await?;
            let _span =
                ozton_runtime::profile::Span::new(ozton_runtime::profile::Stage::MotionTick);
            if started.elapsed() >= timeout {
                return Err(PoseMotionError::StageTimeout);
            }
            let Some(feedback) = self.drive.pose_feedback().map_err(PoseMotionError::Drive)? else {
                log_row!(
                    Trace,
                    "MOTION_RETRY",
                    "command={status},source=tracking_initializing,elapsed_ms={}",
                    started.elapsed().as_millis()
                );
                self.drive.brake().map_err(PoseMotionError::StopFailed)?;
                controller.reset();
                settled = Duration::ZERO;
                progress = Progress::default();
                if feedback_at.elapsed() > self.config.maximum_gap {
                    return Err(PoseMotionError::ControlGap);
                }
                continue;
            };
            let dt = match last_feedback {
                Some(previous) if feedback.time <= previous => {
                    if feedback_at.elapsed() > self.config.maximum_gap {
                        return Err(PoseMotionError::ControlGap);
                    }
                    continue;
                }
                Some(previous) => feedback.time.saturating_sub(previous),
                None => dt,
            };
            if dt > self.config.maximum_gap {
                return Err(PoseMotionError::ControlGap);
            }
            last_feedback = Some(feedback.time);
            feedback_at = Instant::now();
            if !valid_feedback(feedback) {
                return Err(PoseMotionError::InvalidFeedback);
            }
            if (self.drive.footprint().is_some()
                || matches!(
                    target,
                    DifferentialTarget::DriveArc { .. }
                        | DifferentialTarget::TurnToPoint { .. }
                        | DifferentialTarget::DriveToPoint { .. }
                ))
                && !self.drive.pose_trusted()
            {
                return Err(PoseMotionError::UntrustedPose);
            }
            if super::approach::clearance(self.drive, self.config.clearance)?
                .is_some_and(|field| !field.contains_pose(feedback.pose))
            {
                return Err(PoseMotionError::InsufficientClearance);
            }
            let (wheels, ready) = controller.step(target, feedback, dt, self.options);
            settled = if ready { settled + dt } else { Duration::ZERO };
            let complete = ready && settled >= self.options.settle;
            if complete {
                self.drive.brake().map_err(PoseMotionError::StopFailed)?;
            } else {
                self.drive
                    .drive_wheels(wheels.map(Velocity::new::<meter_per_second>))
                    .map_err(PoseMotionError::Drive)?;
            }
            log_row!(
                Trace,
                "MOTION_STEP",
                "command={status},sample_us={},elapsed_ms={},dt_us={},x_m={:.5},y_m={:.5},heading_rad={:.5},forward_mps={:.5},lateral_mps={:.5},yaw_radps={:.5},travel_m={:.5},left_target_mps={:.5},right_target_mps={:.5},complete={complete}",
                feedback.time.as_micros(),
                started.elapsed().as_millis(),
                dt.as_micros(),
                feedback.pose.position.x,
                feedback.pose.position.y,
                feedback.pose.heading.value,
                feedback.forward_velocity.value,
                feedback.lateral_velocity.value,
                feedback.angular_velocity.value,
                feedback.forward_travel.value,
                wheels[0],
                wheels[1]
            );
            self.drive.log_step(self.started.elapsed(), dt);
            if complete {
                return Ok(());
            }
            if progress.stalled(target, feedback, ready, dt, self.config.progress_timeout) {
                return Err(PoseMotionError::NoProgress);
            }
        }
    }

    /// Execute a prepared trajectory with live feedback and a common PID terminal stage.
    pub(super) async fn follow(
        &mut self,
        path: &ozton_math::path::Path,
        follower: crate::pursuit::profiled::Follower,
        reverse: bool,
        target: ozton_math::geometry::Pose,
        clearance: Option<ozton_math::path::clearance::RectangularClearance>,
        timeout: Duration,
    ) -> Result<(), PoseMotionError<D::Error>> {
        use ozton_math::uom::si::{f64::Time, time::second};

        use crate::pursuit::profiled::{self, FollowerSample, Progress as PathProgress};
        let start = Instant::now();
        let mut progress = PathProgress::new(path);
        let mut controller = DifferentialFeedback::new(self.config.tuning).with_config(self.config);
        let mut last = None;
        let mut fresh_at = Instant::now();
        let mut reference_time = 0.0_f64;
        let mut settled = Duration::ZERO;
        let mut best = f64::INFINITY;
        let mut no_progress = Duration::ZERO;
        let mut terminal_turn = false;
        let mut terminal = false;
        let end = path.end();
        let mut options = self.options;
        options.maximum_wheel_speed = options.maximum_wheel_speed.min(path.limits().wheel_speed);
        log_row!(
            Debug,
            "PATH_PLAN",
            "duration_s={:.5},length_m={:.5},reverse={reverse},x_m={:.5},y_m={:.5},heading_rad={:.5}",
            end.time,
            end.distance,
            target.position.x,
            target.position.y,
            target.heading.value
        );
        loop {
            let tick = self.tick().await?;
            let _span =
                ozton_runtime::profile::Span::new(ozton_runtime::profile::Stage::MotionTick);
            if start.elapsed() >= timeout {
                return Err(PoseMotionError::StageTimeout);
            }
            let Some(feedback) = self.drive.pose_feedback().map_err(PoseMotionError::Drive)? else {
                self.drive.brake().map_err(PoseMotionError::StopFailed)?;
                if fresh_at.elapsed() > self.config.maximum_gap {
                    return Err(PoseMotionError::ControlGap);
                }
                continue;
            };
            let dt = match last {
                Some(previous) if feedback.time <= previous => {
                    if fresh_at.elapsed() > self.config.maximum_gap {
                        return Err(PoseMotionError::ControlGap);
                    }
                    continue;
                }
                Some(previous) => feedback.time.saturating_sub(previous),
                None => tick,
            };
            if dt > self.config.maximum_gap {
                return Err(PoseMotionError::ControlGap);
            }
            if !valid_feedback(feedback) {
                return Err(PoseMotionError::InvalidFeedback);
            }
            last = Some(feedback.time);
            fresh_at = Instant::now();
            if !self.drive.pose_trusted() {
                return Err(PoseMotionError::UntrustedPose);
            }
            if clearance.is_some_and(|field| !field.contains_pose(feedback.pose)) {
                return Err(PoseMotionError::InsufficientClearance);
            }
            progress.update(
                feedback.pose.position,
                ozton_math::qty!((options.maximum_wheel_speed.value * dt.as_secs_f64() * 2.0) m),
            );
            let geometric = path.at_distance(ozton_math::qty!((progress.distance()) m));
            let cross = geometric.pose.position.distance(feedback.pose.position);
            if cross > 0.20 {
                return Err(PoseMotionError::PositionUnsettled);
            }
            // Bound time-reference lead to avoid chasing a distant reference after lag.
            reference_time = profiled::advance_reference_time(
                reference_time,
                dt.as_secs_f64(),
                geometric,
                end,
                self.config
                    .lookahead
                    .value
                    .max(self.options.position_tolerance.value * 2.0),
            );
            let sample = FollowerSample {
                pose: feedback.pose,
                elapsed: Time::new::<second>(reference_time),
                period: Time::new::<second>(dt.as_secs_f64()),
            };
            let distance = feedback.pose.position.distance(target.position);
            let heading_error = wrap_heading(target.heading - feedback.pose.heading)
                .value
                .abs();
            if !terminal
                && profiled::is_terminal(
                    &progress,
                    follower,
                    sample,
                    self.options.position_tolerance.value,
                )
            {
                terminal = true;
                best = f64::INFINITY;
                no_progress = Duration::ZERO;
                controller.pid.reset();
            }
            let (wheels, ready) = if terminal {
                let turn = distance <= self.options.position_tolerance.value;
                if turn != terminal_turn {
                    controller.pid.reset();
                    terminal_turn = turn;
                }
                let command = if turn {
                    DifferentialTarget::Turn {
                        heading: target.heading,
                    }
                } else {
                    DifferentialTarget::DriveToPoint {
                        target: target.position,
                        reverse,
                        maximum_speed: options.maximum_wheel_speed,
                    }
                };
                let (wheels, ready) = controller.step(command, feedback, dt, options);
                (
                    wheels,
                    ready && turn && heading_error <= self.options.heading_tolerance.value,
                )
            } else {
                let desired = profiled::command(
                    follower,
                    &progress,
                    sample,
                    reverse,
                    self.options.position_tolerance,
                );
                controller.limit_step(desired, false, dt, options)
            };
            let objective = if terminal {
                distance + self.options.track_width.value * heading_error * 0.5
            } else {
                end.distance - progress.distance()
            };
            if ready || objective < best - 0.002 {
                best = objective;
                no_progress = Duration::ZERO;
            } else {
                no_progress += dt;
            }
            if no_progress >= self.config.progress_timeout {
                return Err(PoseMotionError::NoProgress);
            }
            settled = if ready { settled + dt } else { Duration::ZERO };
            let complete = ready && settled >= self.options.settle;
            if complete {
                self.drive.brake().map_err(PoseMotionError::StopFailed)?;
            } else {
                self.drive
                    .drive_wheels(wheels.map(Velocity::new::<meter_per_second>))
                    .map_err(PoseMotionError::Drive)?;
            }
            log_row!(
                Trace,
                "PATH_TICK",
                "sample_ms={},reference_s={reference_time:.5},progress_m={:.5},cross_m={cross:.5},x_m={:.5},y_m={:.5},heading_rad={:.5},error_m={distance:.5},heading_error_rad={heading_error:.5},left_mps={:.5},right_mps={:.5},terminal={terminal},complete={complete}",
                feedback.time.as_millis(),
                progress.distance(),
                feedback.pose.position.x,
                feedback.pose.position.y,
                feedback.pose.heading.value,
                wheels[0],
                wheels[1]
            );
            self.drive.log_step(self.started.elapsed(), dt);
            if complete {
                return Ok(());
            }
        }
    }

    pub(super) async fn try_distance_reset(
        &mut self,
        timeout: Duration,
    ) -> Result<bool, PoseMotionError<D::Error>> {
        self.drive.brake().map_err(PoseMotionError::StopFailed)?;
        let started = Instant::now();
        self.drive.request_distance_reset();
        loop {
            let dt = self.tick().await?;
            if started.elapsed() >= timeout {
                return Ok(false);
            }
            let feedback = self.drive.pose_feedback().map_err(PoseMotionError::Drive)?;
            self.drive.log_step(self.started.elapsed(), dt);
            if let Some(feedback) = feedback {
                if !valid_feedback(feedback) {
                    return Err(PoseMotionError::InvalidFeedback);
                }
                if feedback.reset_accepted {
                    return Ok(true);
                }
            }
        }
    }

    pub(super) fn finish<T>(
        mut self,
        outcome: Result<T, PoseMotionError<D::Error>>,
    ) -> Result<T, PoseMotionError<D::Error>> {
        self.drive.cancel_distance_reset();
        let result = match (outcome, self.drive.brake()) {
            (outcome, Ok(())) => outcome,
            (Ok(_), Err(stop)) => Err(PoseMotionError::StopFailed(stop)),
            (Err(cause), Err(stop)) => Err(PoseMotionError::StopFailedAfter {
                cause: Box::new(cause),
                stop,
            }),
        };
        self.finished = true;
        result
    }
}

impl<D: PoseDrive> Drop for PoseSession<'_, D> {
    fn drop(&mut self) {
        if !self.finished {
            self.drive.cancel_distance_reset();
            let _ = self.drive.brake();
        }
    }
}

fn valid_feedback(feedback: Feedback) -> bool {
    feedback.pose.is_finite()
        && [
            feedback.forward_travel.value,
            feedback.forward_velocity.value,
            feedback.lateral_velocity.value,
            feedback.angular_velocity.value,
        ]
        .into_iter()
        .all(f64::is_finite)
}

#[derive(Default)]
struct Progress {
    start: Option<Feedback>,
    best: Option<f64>,
    elapsed: Duration,
}

impl Progress {
    fn stalled(
        &mut self,
        target: DifferentialTarget,
        feedback: Feedback,
        ready: bool,
        dt: Duration,
        timeout: Duration,
    ) -> bool {
        let start = *self.start.get_or_insert(feedback);
        let pose = feedback.pose;
        let error = match target {
            DifferentialTarget::DriveToPoint { target, .. } => pose.position.distance(target),
            DifferentialTarget::Turn { heading } => {
                wrap_heading(heading - pose.heading).value.abs()
            }
            DifferentialTarget::TurnToPoint { target } => {
                let delta = target - pose.position;
                if delta.length() <= 1e-9 {
                    0.0
                } else {
                    wrap_heading(ozton_math::geometry::bearing(delta) - pose.heading)
                        .value
                        .abs()
                }
            }
            DifferentialTarget::DriveDistance { meters, .. } => {
                (meters.value - (feedback.forward_travel.value - start.forward_travel.value)).abs()
            }
            DifferentialTarget::DriveArc { radius, sweep, .. } => {
                let remaining = (radius.value * sweep.value.abs()
                    - (feedback.forward_travel.value - start.forward_travel.value))
                    .abs();
                remaining
                    + radius.value
                        * wrap_heading(start.pose.heading + sweep - pose.heading)
                            .value
                            .abs()
            }
        };
        let improvement = match target {
            DifferentialTarget::Turn { .. } | DifferentialTarget::TurnToPoint { .. } => 0.01,
            _ => 0.002,
        };
        let best = self.best.get_or_insert(error);
        if ready || error < *best - improvement {
            *best = error;
            self.elapsed = Duration::ZERO;
        } else {
            self.elapsed += dt;
        }
        self.elapsed >= timeout
    }
}
