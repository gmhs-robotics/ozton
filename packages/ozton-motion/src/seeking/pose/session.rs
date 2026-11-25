use std::time::{Duration, Instant};

use ozton_diagnostics::{log_event, log_row};
use ozton_localization::Feedback;
use ozton_math::{
    geometry::wrap_heading,
    uom::si::{f64::Velocity, velocity::meter_per_second},
};

use super::{
    PathActions, PoseDrive, PoseMotionError,
    safety::{limit_wall_command, safe_feedback},
};
use crate::basic::differential::{
    DifferentialFeedback, DifferentialTarget, DriveOptions, TerminalPose,
};

pub(super) struct Trajectory<'a> {
    pub path: &'a ozton_math::path::Path,
    pub follower: crate::pursuit::profiled::Follower,
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
            previous_tick: Instant::now(),
            started: Instant::now(),
            finished: false,
        })
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
                self.drive.brake().map_err(PoseMotionError::StopFailed)?;
                controller.reset();
                settled = Duration::ZERO;
                progress = Progress::default();
                if feedback_at.elapsed() > self.config.timing.maximum_gap {
                    return Err(PoseMotionError::ControlGap);
                }
                continue;
            };
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
                ))
                && !self.drive.pose_trusted()
            {
                return Err(PoseMotionError::UntrustedPose);
            }
            let clearance = super::approach::clearance(self.drive, self.config.safety.clearance)?;
            let uncertainty = self.drive.velocity_uncertainty();
            let covariance = self.drive.position_covariance();
            if clearance.is_some_and(|field| {
                !safe_feedback(field, feedback, self.config, uncertainty, covariance)
            }) {
                return Err(PoseMotionError::InsufficientClearance);
            }
            let (requested, ready) = controller.step(target, feedback, dt, self.options);
            let wheels = limit_wall_command(
                clearance,
                feedback,
                requested,
                self.options,
                self.config,
                uncertainty,
                covariance,
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
                return Err(PoseMotionError::NoProgress);
            }
        }
    }

    /// Execute a prepared trajectory with live feedback and a common PID terminal stage.
    pub(super) async fn follow(
        &mut self,
        trajectory: Trajectory<'_>,
        actions: &mut PathActions<'_>,
    ) -> Result<(), PoseMotionError<D::Error>> {
        use ozton_math::uom::si::{f64::Time, time::second};

        let Trajectory {
            path,
            follower,
            reverse,
            target,
            clearance,
            timeout,
        } = trajectory;

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
                self.drive.brake().map_err(PoseMotionError::StopFailed)?;
                if fresh_at.elapsed() > self.config.timing.maximum_gap {
                    return Err(PoseMotionError::ControlGap);
                }
                continue;
            };
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
            if clearance.is_some_and(|field| {
                !safe_feedback(field, feedback, self.config, uncertainty, covariance)
            }) {
                return Err(PoseMotionError::InsufficientClearance);
            }
            progress.update(
                feedback.pose.position,
                ozton_math::qty!((options.maximum_wheel_speed.value * dt.as_secs_f64() * 2.0) m),
            );
            let geometric = path.at_distance(ozton_math::qty!((progress.distance()) m));
            let cross = geometric.pose.position.distance(feedback.pose.position);
            if cross > self.config.path.maximum_cross_track_error.value {
                return Err(PoseMotionError::PositionUnsettled);
            }
            actions
                .advance(
                    progress.distance(),
                    (end.time - geometric.time).max(0.0),
                    false,
                )
                .map_err(PoseMotionError::Mechanism)?;
            // Bound time-reference lead to avoid chasing a distant reference after lag.
            reference_time = profiled::advance_reference_time(
                reference_time,
                dt.as_secs_f64(),
                &progress,
                self.config.path.lookahead,
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
                log_event!(Info, "motion", "Trajectory: entering final pose correction");
                best = f64::INFINITY;
                no_progress = Duration::ZERO;
                controller.pid.reset();
            }
            let (requested, ready, objective, phase_changed) = if terminal {
                terminal_pose.step(&mut controller, target, reverse, feedback, dt, options)
            } else {
                let desired = profiled::command(
                    follower,
                    &progress,
                    sample,
                    reverse,
                );
                let (wheels, ready) = controller.limit_step(desired, false, dt, options);
                (wheels, ready, end.distance - progress.distance(), false)
            };
            let wheels = limit_wall_command(
                clearance,
                feedback,
                requested,
                options,
                self.config,
                uncertainty,
                covariance,
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
                return Err(PoseMotionError::NoProgress);
            }
            settled = if ready { settled + dt } else { Duration::ZERO };
            let complete = ready && settled >= self.options.settle;
            if complete || (terminal && (terminal_pose.is_braking() || ready)) {
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
    ) -> Result<bool, PoseMotionError<D::Error>> {
        self.drive.brake().map_err(PoseMotionError::StopFailed)?;
        let started = Instant::now();
        self.drive.request_distance_reset();
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
        let result = match (outcome, self.drive.brake()) {
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
            let stopped = self.drive.brake();
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
        match self.transition(Self::limited(requested, applied), dt) {
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
        } else {
            log_event!(
                Warn,
                "motion",
                "{status}: no progress without current clearance limiting"
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
