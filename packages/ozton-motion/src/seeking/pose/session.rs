use std::time::{Duration, Instant};

use ozton_diagnostics::{log_event, log_row};
use ozton_localization::Feedback;
use ozton_math::{
    geometry::wrap_heading,
    path::clearance::WallContact,
    uom::si::{f64::Velocity, velocity::meter_per_second},
};

use super::{
    PathActions, PoseDrive, PoseMotionError, RangeResetDrive,
    safety::{
        body_rates, contact_released, limit_wall_command, limit_wall_command_at_contact,
        safe_feedback, safe_feedback_at_contact,
    },
};
use crate::basic::differential::{
    DifferentialFeedback, DifferentialTarget, DriveOptions, TerminalPose,
};

pub(super) struct Trajectory<'a> {
    pub path: &'a ozton_math::path::Path,
    pub follower: crate::Follower,
    pub reverse: bool,
    pub target: ozton_math::geometry::Pose,
    pub clearance: Option<ozton_math::path::clearance::RectangularClearance>,
    pub timeout: Duration,
}

pub(super) struct PoseSession<'a, D: PoseDrive> {
    pub(super) drive: &'a mut D,
    pub(super) options: DriveOptions,
    config: crate::MotionConfig,
    previous_tick: Instant,
    started: Instant,
    finished: bool,
    contact: Option<WallContact>,
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
        // Start every independent command with stopped output.
        drive.brake().map_err(PoseMotionError::StopFailed)?;
        Ok(Self {
            drive,
            options,
            config,
            previous_tick: Instant::now(),
            started: Instant::now(),
            finished: false,
            contact: None,
        })
    }

    pub(super) fn with_contact(mut self, contact: Option<WallContact>) -> Self {
        self.contact = contact;
        self
    }

    async fn tick(&mut self) -> Result<Duration, PoseMotionError<D::Error>> {
        let period = self.config.timing.period;
        let deadline = self
            .previous_tick
            .checked_add(period)
            .ok_or(PoseMotionError::Configuration)?;
        self.drive.wait_until(deadline).await;
        let now = Instant::now();
        let dt = now.saturating_duration_since(self.previous_tick);
        self.previous_tick = now;
        ozton_diagnostics::profile::record(
            ozton_diagnostics::profile::Stage::ControlInterval,
            dt,
            period + Duration::from_millis(1),
        );
        if !self.drive.motion_active() {
            return Err(PoseMotionError::Cancelled);
        }
        if dt.is_zero() || dt > self.config.timing.maximum_gap {
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
            DifferentialTarget::DriveToPose { .. } => "Autonomous: Boomerang pose",
            DifferentialTarget::DriveDistance { .. } => "Autonomous: drive distance",
            DifferentialTarget::DriveArc { .. } => "Autonomous: drive arc",
        };
        log_event!(Info, "motion", "{status}: started");
        log_row!(
            Debug,
            "MOTION_COMMAND",
            "state=start,command={status},timeout_ms={}",
            timeout.as_millis()
        );
        let [x, y, heading, travel, radius, sweep] = match target {
            DifferentialTarget::Turn { heading } => [
                f64::NAN,
                f64::NAN,
                heading.value,
                f64::NAN,
                f64::NAN,
                f64::NAN,
            ],
            DifferentialTarget::TurnToPoint { target }
            | DifferentialTarget::DriveToPoint { target, .. } => {
                [target.x, target.y, f64::NAN, f64::NAN, f64::NAN, f64::NAN]
            }
            DifferentialTarget::DriveToPose { target, .. } => [
                target.position.x,
                target.position.y,
                target.heading.value,
                f64::NAN,
                f64::NAN,
                f64::NAN,
            ],
            DifferentialTarget::DriveDistance {
                meters, heading, ..
            } => [
                f64::NAN,
                f64::NAN,
                heading.map_or(f64::NAN, |h| h.value),
                meters.value,
                f64::NAN,
                f64::NAN,
            ],
            DifferentialTarget::DriveArc { radius, sweep, .. } => [
                f64::NAN,
                f64::NAN,
                f64::NAN,
                f64::NAN,
                radius.value,
                sweep.value,
            ],
        };
        log_row!(
            Debug,
            "MOTION_TARGET",
            "command={status},x_m={x:.5},y_m={y:.5},heading_rad={heading:.5},travel_m={travel:.5},radius_m={radius:.5},sweep_rad={sweep:.5}"
        );
        let result = self.run_stage(target, timeout, status).await;
        match &result {
            Ok(()) => {
                log_row!(Debug, "MOTION_COMMAND", "state=complete,command={status}");
                log_event!(Info, "motion", "{status}: settled");
            }
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
        self.previous_tick = started;
        let baseline = self
            .drive
            .pose_feedback()
            .map_err(PoseMotionError::Drive)?
            .map(|f| f.time);
        let mut controller = DifferentialFeedback::new(self.config.tuning).with_config(self.config);
        let mut settled = Duration::ZERO;
        let mut progress = Progress::default();
        let mut governor = GovernorLog::default();
        let mut last_feedback = None;
        let mut feedback_at = Instant::now();
        loop {
            let dt = self.tick().await?;
            let _span = ozton_diagnostics::profile::Span::new(
                ozton_diagnostics::profile::Stage::MotionTick,
            );
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
                self.drive
                    .emergency_brake()
                    .map_err(PoseMotionError::StopFailed)?;
                controller.reset();
                settled = Duration::ZERO;
                progress = Progress::default();
                if feedback_at.elapsed() > self.config.timing.maximum_gap {
                    return Err(PoseMotionError::ControlGap);
                }
                continue;
            };
            if last_feedback.is_none() && baseline.is_some_and(|time| feedback.time <= time) {
                if feedback_at.elapsed() > self.config.timing.maximum_gap {
                    return Err(PoseMotionError::ControlGap);
                }
                continue;
            }
            let dt = match last_feedback {
                Some(previous) if feedback.time <= previous => {
                    if feedback_at.elapsed() > self.config.timing.maximum_gap {
                        return Err(PoseMotionError::ControlGap);
                    }
                    continue;
                }
                Some(previous) => feedback.time.saturating_sub(previous),
                None => dt,
            };
            if dt > self.config.timing.maximum_gap {
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
                        | DifferentialTarget::DriveToPose { .. }
                ))
                && !self.drive.pose_trusted()
            {
                return Err(PoseMotionError::UntrustedPose);
            }
            let clearance = super::approach::clearance(self.drive, self.config.safety.clearance)?;
            let uncertainty = self.drive.velocity_uncertainty();
            let covariance = self.drive.position_covariance();
            let previous = controller.applied_wheels();
            let previous_rates = body_rates(previous, self.options.track_width.value);
            if self.contact.is_some()
                && clearance.is_some_and(|field| {
                    contact_released(
                        field,
                        feedback,
                        self.config,
                        uncertainty,
                        covariance,
                        previous_rates,
                    )
                })
            {
                self.contact = None;
                self.drive.release_wall_contact();
                log_event!(
                    Info,
                    "clearance",
                    "Surveyed contact departure cleared; ordinary governor restored"
                );
            }
            if clearance.is_some_and(|field| {
                !safe_feedback_at_contact(
                    field,
                    feedback,
                    self.config,
                    uncertainty,
                    covariance,
                    previous_rates,
                    self.contact,
                )
            }) {
                return Err(PoseMotionError::InsufficientClearance);
            }
            let (requested, ready) = controller.step(target, feedback, dt, self.options);
            let wheels = limit_wall_command_at_contact(
                clearance,
                feedback,
                requested,
                self.options,
                self.config,
                uncertainty,
                covariance,
                previous,
                self.contact,
            );
            controller.record_applied_wheels(wheels);
            settled = if ready { settled + dt } else { Duration::ZERO };
            let complete = ready && settled >= self.options.settle;
            if complete {
                self.drive.brake().map_err(PoseMotionError::StopFailed)?;
            } else {
                self.drive
                    .drive_wheels(wheels.map(Velocity::new::<meter_per_second>))
                    .map_err(PoseMotionError::Drive)?;
            }
            governor.observe(status, requested, wheels, dt);
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
            if progress.stalled(
                target,
                feedback,
                ready,
                dt,
                self.config.timing.progress_timeout,
            ) {
                governor.report_stall(status, requested, wheels);
                return Err(if GovernorLog::limited(requested, wheels) {
                    PoseMotionError::InsufficientClearance
                } else {
                    PoseMotionError::NoProgress
                });
            }
        }
    }

    /// Execute a prepared trajectory with live feedback and a common PID terminal stage.
    pub(super) async fn follow(
        &mut self,
        trajectory: Trajectory<'_>,
        actions: &mut PathActions<'_>,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let Trajectory {
            path,
            follower,
            reverse,
            target,
            clearance,
            timeout,
        } = trajectory;

        use crate::pursuit::profiled::ProfiledFollower;
        let start = Instant::now();
        self.previous_tick = start;
        let baseline = self
            .drive
            .pose_feedback()
            .map_err(PoseMotionError::Drive)?
            .map(|f| f.time);
        let mut follower = ProfiledFollower::new(path, follower);
        let mut controller = DifferentialFeedback::new(self.config.tuning).with_config(self.config);
        let mut last = None;
        let mut fresh_at = Instant::now();
        let mut settled = Duration::ZERO;
        let mut best = f64::INFINITY;
        let mut no_progress = Duration::ZERO;
        let mut terminal_pose = TerminalPose::default();
        let mut terminal = false;
        let mut governor = GovernorLog::default();
        log_event!(Info, "motion", "Trajectory: started");
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
            let _span = ozton_diagnostics::profile::Span::new(
                ozton_diagnostics::profile::Stage::MotionTick,
            );
            if start.elapsed() >= timeout {
                return Err(PoseMotionError::StageTimeout);
            }
            let Some(feedback) = self.drive.pose_feedback().map_err(PoseMotionError::Drive)? else {
                self.drive
                    .emergency_brake()
                    .map_err(PoseMotionError::StopFailed)?;
                controller.reset();
                if fresh_at.elapsed() > self.config.timing.maximum_gap {
                    return Err(PoseMotionError::ControlGap);
                }
                continue;
            };
            if last.is_none() && baseline.is_some_and(|time| feedback.time <= time) {
                if fresh_at.elapsed() > self.config.timing.maximum_gap {
                    return Err(PoseMotionError::ControlGap);
                }
                continue;
            }
            let dt = match last {
                Some(previous) if feedback.time <= previous => {
                    if fresh_at.elapsed() > self.config.timing.maximum_gap {
                        return Err(PoseMotionError::ControlGap);
                    }
                    continue;
                }
                Some(previous) => feedback.time.saturating_sub(previous),
                None => tick,
            };
            if dt > self.config.timing.maximum_gap {
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
            let uncertainty = self.drive.velocity_uncertainty();
            let covariance = self.drive.position_covariance();
            let previous = controller.applied_wheels();
            if clearance.is_some_and(|field| {
                !safe_feedback(
                    field,
                    feedback,
                    self.config,
                    uncertainty,
                    covariance,
                    body_rates(previous, self.options.track_width.value),
                )
            }) {
                return Err(PoseMotionError::InsufficientClearance);
            }
            let observation = follower.observe(
                feedback.pose,
                dt,
                ozton_math::qty!((options.maximum_wheel_speed.value * dt.as_secs_f64() * 2.0) m),
                self.config.path.lookahead,
            );
            let cross = observation.cross_track;
            if cross > self.config.path.maximum_cross_track_error.value {
                return Err(PoseMotionError::PositionUnsettled);
            }
            actions
                .advance(observation.distance, observation.remaining_time, false)
                .map_err(PoseMotionError::Mechanism)?;
            let reference_time = observation.reference_time;
            let distance = feedback.pose.position.distance(target.position);
            let heading_error = wrap_heading(target.heading - feedback.pose.heading)
                .value
                .abs();
            if !terminal && follower.is_terminal(self.options.position_tolerance.value) {
                terminal = true;
                log_event!(Info, "motion", "Trajectory: entering final pose correction");
                best = f64::INFINITY;
                no_progress = Duration::ZERO;
                controller.reset_feedback();
            }
            let (requested, ready, objective, phase_changed) = if terminal {
                terminal_pose.step(&mut controller, target, reverse, feedback, dt, options)
            } else {
                let desired = follower.command(feedback.pose, dt, reverse);
                let (wheels, ready) = controller.limit_step(desired, false, dt, options);
                (wheels, ready, end.distance - observation.distance, false)
            };
            let wheels = limit_wall_command(
                clearance,
                feedback,
                requested,
                options,
                self.config,
                uncertainty,
                covariance,
                previous,
            );
            controller.record_applied_wheels(wheels);
            if phase_changed {
                best = f64::INFINITY;
                no_progress = Duration::ZERO;
            }
            if ready || objective < best - 0.002 {
                best = objective;
                no_progress = Duration::ZERO;
            } else {
                no_progress += dt;
            }
            if no_progress >= self.config.timing.progress_timeout {
                governor.report_stall("Trajectory", requested, wheels);
                return Err(if GovernorLog::limited(requested, wheels) {
                    PoseMotionError::InsufficientClearance
                } else {
                    PoseMotionError::NoProgress
                });
            }
            settled = if ready { settled + dt } else { Duration::ZERO };
            let complete = ready && settled >= self.options.settle;
            if terminal && terminal_pose.is_braking() {
                self.drive
                    .emergency_brake()
                    .map_err(PoseMotionError::StopFailed)?;
            } else if complete || (terminal && ready) {
                self.drive.brake().map_err(PoseMotionError::StopFailed)?;
            } else {
                self.drive
                    .drive_wheels(wheels.map(Velocity::new::<meter_per_second>))
                    .map_err(PoseMotionError::Drive)?;
            }
            governor.observe("Trajectory", requested, wheels, dt);
            log_row!(
                Trace,
                "PATH_TICK",
                "sample_ms={},reference_s={reference_time:.5},progress_m={:.5},cross_m={cross:.5},x_m={:.5},y_m={:.5},heading_rad={:.5},error_m={distance:.5},heading_error_rad={heading_error:.5},left_mps={:.5},right_mps={:.5},terminal={terminal},complete={complete},dt_us={},requested_left_mps={:.5},requested_right_mps={:.5},forward_mps={:.5},lateral_mps={:.5},yaw_radps={:.5}",
                feedback.time.as_millis(),
                observation.distance,
                feedback.pose.position.x,
                feedback.pose.position.y,
                feedback.pose.heading.value,
                wheels[0],
                wheels[1],
                dt.as_micros(),
                requested[0],
                requested[1],
                feedback.forward_velocity.value,
                feedback.lateral_velocity.value,
                feedback.angular_velocity.value
            );
            self.drive.log_step(self.started.elapsed(), dt);
            if complete {
                actions
                    .advance(end.distance, 0.0, true)
                    .map_err(PoseMotionError::Mechanism)?;
                log_event!(Info, "motion", "Trajectory: settled");
                return Ok(());
            }
        }
    }

    pub(super) async fn try_distance_reset(
        &mut self,
        timeout: Duration,
    ) -> Result<bool, PoseMotionError<D::Error>>
    where
        D: RangeResetDrive,
    {
        self.drive.brake().map_err(PoseMotionError::StopFailed)?;
        let started = Instant::now();
        self.previous_tick = started;
        <D as RangeResetDrive>::request_distance_reset(self.drive);
        log_event!(
            Info,
            "localization",
            "Stationary distance correction: requested"
        );
        loop {
            let dt = self.tick().await?;
            if started.elapsed() >= timeout {
                log_event!(
                    Warn,
                    "localization",
                    "Stationary distance correction: no accepted fix before timeout"
                );
                return Ok(false);
            }
            let feedback = self.drive.pose_feedback().map_err(PoseMotionError::Drive)?;
            self.drive.log_step(self.started.elapsed(), dt);
            if let Some(feedback) = feedback {
                if !valid_feedback(feedback) {
                    return Err(PoseMotionError::InvalidFeedback);
                }
                if feedback.reset_accepted {
                    log_event!(
                        Info,
                        "localization",
                        "Stationary distance correction: accepted"
                    );
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
        if self.contact.take().is_some() {
            self.drive.release_wall_contact();
        }
        let stop = if outcome.is_ok() {
            self.drive.brake()
        } else {
            self.drive.emergency_brake()
        };
        let result = match (outcome, stop) {
            (outcome, Ok(())) => outcome,
            (Ok(_), Err(stop)) => Err(PoseMotionError::StopFailed(stop)),
            (Err(cause), Err(stop)) => Err(PoseMotionError::StopFailedAfter {
                cause: Box::new(cause),
                stop,
            }),
        };
        self.finished = true;
        if let Err(error) = &result {
            log_event!(Warn, "motion", "Motion ended: {}", error.reason());
        }
        result
    }
}

impl<D: PoseDrive> Drop for PoseSession<'_, D> {
    fn drop(&mut self) {
        if !self.finished {
            self.drive.cancel_distance_reset();
            if self.contact.take().is_some() {
                self.drive.release_wall_contact();
            }
            let stopped = self.drive.emergency_brake();
            if stopped.is_ok() {
                log_event!(Info, "motion", "Motion interrupted; brake requested");
            } else {
                log_event!(Error, "motion", "Motion interrupted; brake request failed");
            }
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
            DifferentialTarget::DriveToPose { target, .. } => {
                pose.position.distance(target.position)
                    + 0.1 * wrap_heading(target.heading - pose.heading).value.abs()
            }
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

/// USB reports sustained governor transitions, not per-tick measurements.
/// Reporting thresholds do not change the physical safety constraints.
#[derive(Default)]
struct GovernorLog {
    active: bool,
    pending: Duration,
    limited_ticks: u64,
    total_ticks: u64,
}

impl GovernorLog {
    fn limited(requested: [f64; 2], applied: [f64; 2]) -> bool {
        let peak = requested[0].abs().max(requested[1].abs());
        let difference = (requested[0] - applied[0])
            .abs()
            .max((requested[1] - applied[1]).abs());
        peak > 0.0 && difference > peak * 0.05
    }

    fn transition(&mut self, limited: bool, dt: Duration) -> Option<bool> {
        if limited == self.active {
            self.pending = Duration::ZERO;
            return None;
        }
        self.pending += dt;
        if self.pending < Duration::from_millis(250) {
            return None;
        }
        self.pending = Duration::ZERO;
        self.active = limited;
        Some(limited)
    }

    fn observe(&mut self, status: &str, requested: [f64; 2], applied: [f64; 2], dt: Duration) {
        let limited = Self::limited(requested, applied);
        self.total_ticks += 1;
        if limited {
            self.limited_ticks += 1;
        }
        match self.transition(limited, dt) {
            Some(true) => log_event!(
                Info,
                "clearance",
                "{status}: wall/obstacle speed governor active"
            ),
            Some(false) => log_event!(Info, "clearance", "{status}: speed governor released"),
            None => {}
        }
    }

    fn report_stall(&self, status: &str, requested: [f64; 2], applied: [f64; 2]) {
        if Self::limited(requested, applied) {
            log_event!(
                Warn,
                "motion",
                "{status}: no progress while clearance governor limits output"
            );
        } else if self.limited_ticks > 0 {
            log_event!(
                Warn,
                "motion",
                "{status}: no progress; intermittent clearance limiting ({}/{} samples)",
                self.limited_ticks,
                self.total_ticks
            );
        } else {
            log_event!(
                Warn,
                "motion",
                "{status}: no progress without clearance limiting"
            );
        }
    }
}

#[cfg(test)]
mod governor_log_tests {
    use super::*;

    #[test]
    fn only_sustained_transitions_are_reported() {
        let mut report = GovernorLog::default();
        let dt = Duration::from_millis(100);
        assert!(GovernorLog::limited([0.2, -0.2], [0.1, -0.1]));
        assert!(!GovernorLog::limited([0.2, -0.2], [0.2, -0.2]));
        assert_eq!(report.transition(true, dt), None);
        assert_eq!(report.transition(false, dt), None);
        assert_eq!(report.transition(true, dt), None);
        assert_eq!(report.transition(true, dt), None);
        assert_eq!(report.transition(true, dt), Some(true));
        assert_eq!(report.transition(true, dt), None);
        assert_eq!(report.transition(false, dt), None);
        assert_eq!(report.transition(false, dt), None);
        assert_eq!(report.transition(false, dt), Some(false));
    }
}
