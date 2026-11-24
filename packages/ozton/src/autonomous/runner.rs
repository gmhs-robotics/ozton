//! Bounded execution for programmed autonomous, including cancellation by drop.
use std::time::{Duration, Instant};

use ozton_math::{
    geometry::{Pose, wrap_heading},
    uom::si::{
        f64::{Length, Velocity},
        velocity::meter_per_second,
    },
};
use ozton_runtime::{clock::CommandClock, log_event, log_row};
use vexide::competition::{self, CompetitionMode};

use super::{
    Alignment, Command, CommandError, DriveOptions, DriveTuning, Feedback, command::State,
};

/// Connects an application robot to the same executor used by simulated robots.
pub trait MotionPlatform {
    type Error;

    /// Capture tracking once. None is a stopped initialization/rebaseline tick.
    fn sample(&mut self) -> Result<Option<Feedback>, Self::Error>;
    fn update_subsystems(&mut self, dt: Duration) -> Result<(), Self::Error>;
    fn drive(&mut self, meters_per_second: [Velocity; 2], dt: Duration) -> Result<(), Self::Error>;
    fn stop_drive(&mut self) -> Result<(), Self::Error>;
    /// Stop every output, even when an individual device fails.
    fn stop(&mut self) -> Result<(), Self::Error>;
    fn set_pose(&mut self, pose: Pose) -> Result<(), Self::Error>;
    /// True only after explicit field registration, and false after lost odometry.
    fn pose_trusted(&self) -> bool {
        false
    }
    /// Only classify transient device failures as retryable.
    fn recoverable(&self, _error: &Self::Error) -> bool {
        false
    }
    /// May an optional stage return false after this failure and a full stop?
    /// The route must explicitly select a fallback; default is conservative.
    fn can_skip_failure(&self, _error: &Self::Error) -> bool {
        false
    }
    /// Remove output while retaining goals for a bounded sensor retry.
    fn pause(&mut self) -> Result<(), Self::Error> {
        self.stop()
    }
    fn request_distance_reset(&mut self);
    fn cancel_distance_reset(&mut self) {}
    /// Display the current operation when the robot has a competition UI.
    fn show_status(&mut self, _status: &str) {}
    fn log_step(
        &mut self,
        _elapsed: Duration,
        _dt: Duration,
        _progress: Length,
        _wheels: [Velocity; 2],
    ) {
    }
}

/// Reuse one runner across commands to share motion settings.
/// Dropping it stops drive and registered subsystems, including when vexide cancels autonomous.
pub struct Runner<'a, R: MotionPlatform> {
    robot: &'a mut R,
    clock: CommandClock,
    options: DriveOptions,
    tuning: DriveTuning,
    #[cfg(feature = "logging")]
    started: Instant,
    max_gap: Duration,
    failed: bool,
    stop_on_drop: bool,
}

impl<'a, R: MotionPlatform> Runner<'a, R> {
    pub fn new(
        robot: &'a mut R,
        options: DriveOptions,
        period: Duration,
        max_gap: Duration,
    ) -> Result<Self, CommandError<R::Error>> {
        if !Self::valid_configuration(options, period, max_gap) {
            return Err(CommandError::Configuration);
        }
        Ok(Self {
            robot,
            clock: CommandClock::new(period).ok_or(CommandError::Configuration)?,
            options,
            tuning: DriveTuning::STANDARD,
            #[cfg(feature = "logging")]
            started: Instant::now(),
            max_gap,
            failed: false,
            stop_on_drop: true,
        })
    }

    /// Use these PID gains for subsequent commands in this session.
    pub fn with_tuning(mut self, tuning: DriveTuning) -> Result<Self, CommandError<R::Error>> {
        if !tuning.valid() {
            return Err(CommandError::Configuration);
        }
        self.tuning = tuning;
        Ok(self)
    }

    /// Finish a single command while leaving retained subsystem goals in place.
    /// Cancellation or a failed brake still takes the full stop path on drop.
    pub(super) fn finish_command(mut self) -> Result<(), CommandError<R::Error>> {
        self.robot.stop_drive().map_err(CommandError::StopFailed)?;
        self.stop_on_drop = false;
        Ok(())
    }

    pub fn valid_configuration(options: DriveOptions, period: Duration, max_gap: Duration) -> bool {
        options.valid() && CommandClock::new(period).is_some() && max_gap >= period
    }

    pub(super) fn check_active(&mut self) -> Result<(), CommandError<R::Error>> {
        if self.failed {
            return Err(self.finish_error(CommandError::Cancelled));
        }
        Ok(())
    }

    /// Inspect the robot without exclusive access to the runner.
    pub fn robot_ref(&self) -> &R {
        self.robot
    }

    /// Change retained mechanism goals between commands.
    pub fn robot(&mut self) -> &mut R {
        self.robot
    }

    /// Physical drive options used for this session.
    pub fn options(&self) -> DriveOptions {
        self.options
    }

    /// Wait at zero drive output for a data-dependent condition. The predicate
    /// can capture calibrated thresholds; every tick still services mechanisms,
    /// tracking and cancellation.
    pub async fn wait_until(
        &mut self,
        ready: impl FnMut(&R) -> bool,
        stable: Duration,
        timeout: Duration,
    ) -> Result<(), CommandError<R::Error>> {
        self.wait_until_while(ready, stable, timeout, || {
            competition::mode() == CompetitionMode::Autonomous
        })
        .await
    }

    /// Captured predicate with an explicit enable source for simulation/practice.
    pub async fn wait_until_while(
        &mut self,
        mut ready: impl FnMut(&R) -> bool,
        stable: Duration,
        timeout: Duration,
        enabled: impl FnMut() -> bool,
    ) -> Result<(), CommandError<R::Error>> {
        self.run_while(
            Command::WaitFor {
                ready: &mut ready,
                stable,
            },
            timeout,
            enabled,
        )
        .await
    }

    pub async fn run(
        &mut self,
        command: Command<'_, R>,
        timeout: Duration,
    ) -> Result<(), CommandError<R::Error>> {
        self.run_while(command, timeout, || {
            competition::mode() == CompetitionMode::Autonomous
        })
        .await
    }

    /// Supplies a custom enable predicate for testing or a practice mode.
    pub async fn run_while(
        &mut self,
        command: Command<'_, R>,
        timeout: Duration,
        enabled: impl FnMut() -> bool,
    ) -> Result<(), CommandError<R::Error>> {
        self.run_with_policy(command, timeout, enabled, false)
            .await
            .map(|_| ())
    }

    /// Attempt an optional stage. A bounded failure stops outputs and returns false;
    /// caller can choose a different command. Cancellation and route expiry remain terminal.
    pub async fn try_run(
        &mut self,
        command: Command<'_, R>,
        timeout: Duration,
    ) -> Result<bool, CommandError<R::Error>> {
        self.try_run_while(command, timeout, || {
            competition::mode() == CompetitionMode::Autonomous
        })
        .await
    }
    pub async fn try_run_while(
        &mut self,
        command: Command<'_, R>,
        timeout: Duration,
        enabled: impl FnMut() -> bool,
    ) -> Result<bool, CommandError<R::Error>> {
        let result = self.execute(command, timeout, enabled, false).await;
        let error = match result {
            Ok(completed) => return Ok(completed),
            Err(error) => error,
        };
        let can_skip = match &error {
            CommandError::CommandTimeout
            | CommandError::NoProgress
            | CommandError::PathLost
            | CommandError::UntrustedPose => true,
            CommandError::Robot(error) => self.robot.can_skip_failure(error),
            _ => false,
        };
        self.robot.cancel_distance_reset();
        let stopped = self.robot.stop();
        if can_skip && stopped.is_ok() {
            #[cfg(feature = "logging")]
            log_event!(
                Warn,
                "motion",
                "Optional command stopped ({}). Continuing.",
                error.code()
            );
            return Ok(false);
        }
        self.failed = true;
        Err(match stopped {
            Ok(()) => error,
            Err(stop) => CommandError::StopFailedAfter {
                cause: Box::new(error),
                stop,
            },
        })
    }

    /// Returns false after a clean alignment timeout. The next command can run
    /// a fallback while autonomous remains active.
    pub async fn try_align(
        &mut self,
        alignment: Alignment,
        timeout: Duration,
    ) -> Result<bool, CommandError<R::Error>> {
        self.try_align_while(alignment, timeout, || {
            competition::mode() == CompetitionMode::Autonomous
        })
        .await
    }

    pub async fn try_align_while(
        &mut self,
        alignment: Alignment,
        timeout: Duration,
        enabled: impl FnMut() -> bool,
    ) -> Result<bool, CommandError<R::Error>> {
        self.run_with_policy(Command::Align(alignment), timeout, enabled, true)
            .await
    }

    /// Optional DSR: a clean timeout returns false so the route can use odometry.
    pub async fn try_distance_reset(
        &mut self,
        timeout: Duration,
    ) -> Result<bool, CommandError<R::Error>> {
        self.try_distance_reset_while(timeout, || {
            competition::mode() == CompetitionMode::Autonomous
        })
        .await
    }
    pub async fn try_distance_reset_while(
        &mut self,
        timeout: Duration,
        enabled: impl FnMut() -> bool,
    ) -> Result<bool, CommandError<R::Error>> {
        self.run_with_policy(Command::DistanceReset, timeout, enabled, true)
            .await
    }

    async fn run_with_policy(
        &mut self,
        command: Command<'_, R>,
        timeout: Duration,
        enabled: impl FnMut() -> bool,
        recover_timeout: bool,
    ) -> Result<bool, CommandError<R::Error>> {
        let result = self
            .execute(command, timeout, enabled, recover_timeout)
            .await;
        result.map_err(|error| self.finish_error(error))
    }

    pub(super) fn finish_error(&mut self, error: CommandError<R::Error>) -> CommandError<R::Error> {
        self.failed = true;
        self.robot.cancel_distance_reset();
        match self.robot.stop() {
            Ok(()) => error,
            Err(stop) => CommandError::StopFailedAfter {
                cause: Box::new(error),
                stop,
            },
        }
    }

    async fn execute(
        &mut self,
        mut command: Command<'_, R>,
        timeout: Duration,
        mut enabled: impl FnMut() -> bool,
        recover_timeout: bool,
    ) -> Result<bool, CommandError<R::Error>> {
        if self.failed {
            return Err(CommandError::Cancelled);
        }
        let status = command.status();
        if timeout.is_zero() || !command.valid(self.options) {
            log_event!(
                Error,
                "motion",
                "{status}: invalid command settings (timeout {} ms, track width {:.3} m, maximum wheel speed {:.3} m/s)",
                timeout.as_millis(),
                self.options.track_width.value,
                self.options.maximum_wheel_speed.value
            );
            return Err(CommandError::Configuration);
        }
        let mut active = ActiveCommand {
            robot: self.robot,
            completed: false,
        };
        let outcome = async {
            active.robot.show_status(status);
            log_event!(Info, "motion", "{status} (timeout {:.2}s)", timeout.as_secs_f64());
            log_row!(Debug, "MOTION_COMMAND", "state=start,command={status},timeout_ms={}", timeout.as_millis());
            #[cfg(feature = "logging")]
            if let Some((path, reverse)) = command.path_start() {
                log_row!(
                    Debug, "PATH_PLAN",
                    "start_x_m={:.5},start_y_m={:.5},start_heading_rad={:.5},end_x_m={:.5},end_y_m={:.5},end_heading_rad={:.5},length_m={:.5},duration_ms={:.0},reverse={reverse}",
                    path.start().pose.position.x, path.start().pose.position.y, path.start().pose.heading.value,
                    path.end().pose.position.x, path.end().pose.position.y, path.end().pose.heading.value,
                    path.end().distance, path.end().time * 1000.0
                );
            }
            let started = Instant::now();
            let mut state = State::new(self.tuning);
            let mut initialized = false;
            let mut retry_since = None;
            let mut mechanism_retry_since = None;
            let mut output_retry_since = None;
            let mut progress_pose: Option<Pose> = None;
            let mut progress_elapsed = Duration::ZERO;
            loop {
                let dt = self.clock.tick().await;
                let elapsed = started.elapsed();
                if !enabled() {
                    return Err(CommandError::Cancelled);
                }
                if dt.is_zero() || dt > self.max_gap {
                    log_event!(Error, "motion", "{status}: control tick was {:.1} ms, limit {:.1} ms", dt.as_secs_f64() * 1000.0, self.max_gap.as_secs_f64() * 1000.0);
                    return Err(CommandError::ControlGap);
                }
                if elapsed >= timeout {
                    if recover_timeout {
                        if matches!(command, Command::DistanceReset) {
                            active.robot.cancel_distance_reset();
                        }
                        active
                            .robot
                            .stop_drive()
                            .map_err(CommandError::StopFailed)?;
                        return Ok(false);
                    }
                    return Err(CommandError::CommandTimeout);
                }
                if matches!(command, Command::DistanceReset) {
                    active.robot.request_distance_reset();
                }
                let feedback = match active.robot.sample() {
                    Ok(value) => {
                        retry_since = None;
                        value
                    }
                    Err(error) if active.robot.recoverable(&error) => {
                        log_row!(Trace, "MOTION_RETRY", "command={status},source=tracking,elapsed_ms={}", elapsed.as_millis());
                        let since = *retry_since.get_or_insert(Instant::now());
                        active.robot.pause().map_err(CommandError::StopFailed)?;
                        if since.elapsed() >= Duration::from_millis(300) {
                            return Err(CommandError::Robot(error));
                        }
                        state.pause();
                        progress_pose = None;
                        progress_elapsed = Duration::ZERO;
                        continue;
                    }
                    Err(error) => return Err(CommandError::Robot(error)),
                };
                let Some(feedback) = feedback else {
                    log_row!(Trace, "MOTION_RETRY", "command={status},source=tracking_initializing,elapsed_ms={}", elapsed.as_millis());
                    active.robot.pause().map_err(CommandError::StopFailed)?;
                    state.pause();
                    progress_pose = None;
                    progress_elapsed = Duration::ZERO;
                    continue;
                };
                if !feedback.pose.is_finite()
                    || ![
                        feedback.forward_travel.value,
                        feedback.forward_velocity.value,
                        feedback.lateral_velocity.value,
                        feedback.angular_velocity.value,
                    ]
                    .iter()
                    .all(|value| value.is_finite())
                {
                    return Err(CommandError::InvalidFeedback);
                }
                if command.requires_field_pose() && !active.robot.pose_trusted() {
                    return Err(CommandError::UntrustedPose);
                }
                if !initialized {
                    if let Some((path, reverse)) = command.path_start() {
                        let start = path.start().pose;
                        let heading = start.heading
                            + ozton_math::angle!(if reverse {
                                core::f64::consts::PI
                            } else {
                                0.0
                            }, rad);
                        if feedback.pose.position.distance(start.position) > 0.15
                            || wrap_heading(feedback.pose.heading - heading).value.abs() > 1.05
                        {
                            log_event!(Error, "path", "Start mismatch. Pose ({:.3}, {:.3}) m at {:.3} rad, expected ({:.3}, {:.3}) m at {:.3} rad",
                                feedback.pose.position.x, feedback.pose.position.y, feedback.pose.heading.value,
                                start.position.x, start.position.y, heading.value
                            );
                            return Err(CommandError::PathStart);
                        }
                    }
                    initialized = true;
                }
                let step = state.step(
                    &mut command,
                    active.robot,
                    feedback,
                    elapsed,
                    dt,
                    self.options,
                );
                log_row!(
                    Trace, "MOTION_STEP",
                    "command={status},elapsed_ms={},dt_us={},x_m={:.5},y_m={:.5},heading_rad={:.5},progress_m={:.5},left_target_mps={:.5},right_target_mps={:.5},complete={},lost={}",
                    elapsed.as_millis(), dt.as_micros(), feedback.pose.position.x, feedback.pose.position.y,
                    feedback.pose.heading.value, step.progress, step.wheels[0], step.wheels[1], step.complete, step.lost
                );
                if step.lost {
                    log_event!(Error, "path", "Follower lost path at {:.3} m progress. Pose ({:.3}, {:.3}) m at {:.3} rad",
                        step.progress, feedback.pose.position.x, feedback.pose.position.y, feedback.pose.heading.value
                    );
                    return Err(CommandError::PathLost);
                }
                if let Err(error) = active.robot.update_subsystems(dt) {
                    if active.robot.recoverable(&error) {
                        log_row!(Trace, "MOTION_RETRY", "command={status},source=mechanism,elapsed_ms={}", elapsed.as_millis());
                        let since = *mechanism_retry_since.get_or_insert(Instant::now());
                        active.robot.pause().map_err(CommandError::StopFailed)?;
                        if since.elapsed() < Duration::from_millis(300) {
                            state.pause();
                            progress_pose = None;
                            progress_elapsed = Duration::ZERO;
                            continue;
                        }
                    }
                    return Err(CommandError::Robot(error));
                }
                mechanism_retry_since = None;
                if step.complete || step.wheels == [0.0; 2] {
                    active
                        .robot
                        .stop_drive()
                        .map_err(CommandError::StopFailed)?;
                } else {
                    match active
                        .robot
                        .drive(step.wheels.map(Velocity::new::<meter_per_second>), dt)
                    {
                        Ok(()) => output_retry_since = None,
                        Err(error) if active.robot.recoverable(&error) => {
                            log_row!(Trace, "MOTION_RETRY", "command={status},source=drive_output,elapsed_ms={}", elapsed.as_millis());
                            let since = *output_retry_since.get_or_insert(Instant::now());
                            active.robot.pause().map_err(CommandError::StopFailed)?;
                            if since.elapsed() >= Duration::from_millis(300) {
                                return Err(CommandError::Robot(error));
                            }
                            state.pause();
                            progress_pose = None;
                            progress_elapsed = Duration::ZERO;
                            continue;
                        }
                        Err(error) => return Err(CommandError::Robot(error)),
                    }
                }
                if step.wheels.iter().any(|v| v.abs() > 0.08) {
                    let anchor = *progress_pose.get_or_insert(feedback.pose);
                    if feedback.pose.position.distance(anchor.position) > 0.008
                        || wrap_heading(feedback.pose.heading - anchor.heading)
                            .value
                            .abs()
                            > 0.025
                    {
                        progress_pose = Some(feedback.pose);
                        progress_elapsed = Duration::ZERO;
                    } else {
                        progress_elapsed += dt;
                        if progress_elapsed >= Duration::from_millis(800) {
                            log_event!(Error, "motion", "{status}: no progress for {} ms. Pose ({:.3}, {:.3}) m, wheel targets {:.3}/{:.3} m/s",
                                progress_elapsed.as_millis(), feedback.pose.position.x, feedback.pose.position.y,
                                step.wheels[0], step.wheels[1]
                            );
                            return Err(CommandError::NoProgress);
                        }
                    }
                } else {
                    progress_pose = None;
                    progress_elapsed = Duration::ZERO;
                }
                #[cfg(feature = "logging")]
                active.robot.log_step(
                    self.started.elapsed(),
                    dt,
                    ozton_math::length!(step.progress, m),
                    step.wheels.map(Velocity::new::<meter_per_second>),
                );
                if step.complete {
                    if let Command::Contact(contact) = command {
                        active
                            .robot
                            .set_pose(contact.known_pose)
                            .map_err(CommandError::Robot)?;
                    }
                    return Ok(true);
                }
            }
        }
        .await;
        #[cfg(feature = "logging")]
        match &outcome {
            Ok(true) => log_row!(Debug, "MOTION_COMMAND", "state=complete,command={status}"),
            Ok(false) => log_event!(
                Warn,
                "motion",
                "{status}: timed out after {:.2}s, continuing",
                timeout.as_secs_f64()
            ),
            Err(error) => log_row!(
                Debug,
                "MOTION_COMMAND",
                "state=failed,command={status},reason={}",
                error.code()
            ),
        }
        // Regular exits are finalized by the command policy. The guard owns cancellation cleanup.
        active.completed = true;
        outcome
    }
}

struct ActiveCommand<'a, R: MotionPlatform> {
    robot: &'a mut R,
    completed: bool,
}

impl<R: MotionPlatform> Drop for ActiveCommand<'_, R> {
    fn drop(&mut self) {
        if !self.completed {
            self.robot.cancel_distance_reset();
            let _ = self.robot.stop();
        }
    }
}

impl<R: MotionPlatform> Drop for Runner<'_, R> {
    fn drop(&mut self) {
        if self.stop_on_drop {
            self.robot.cancel_distance_reset();
            let _ = self.robot.stop();
        }
    }
}
