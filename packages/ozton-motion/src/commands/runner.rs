//! Bounded execution for programmed autonomous, including cancellation by drop.
use std::time::{Duration, Instant};

use ozton_math::{
    geometry::{Pose, wrap_heading},
    uom::si::{
        angle::radian,
        f64::{Angle, Length, Velocity},
        length::meter,
        velocity::meter_per_second,
    },
};
use ozton_runtime::clock::CommandClock;
use vexide::competition::{self, CompetitionMode};

use super::{Alignment, Command, CommandError, DriveOptions, Feedback, command::State};

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

/// Reuse one runner across all commands to enforce an overall route deadline.
/// Dropping it stops drive and registered subsystems, including when vexide cancels autonomous.
pub struct Runner<'a, R: MotionPlatform> {
    robot: &'a mut R,
    clock: CommandClock,
    options: DriveOptions,
    started: Instant,
    route_timeout: Duration,
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
        route_timeout: Duration,
    ) -> Result<Self, CommandError<R::Error>> {
        if !Self::valid_configuration(options, period, max_gap, route_timeout) {
            return Err(CommandError::Configuration);
        }
        Ok(Self {
            robot,
            clock: CommandClock::new(period).ok_or(CommandError::Configuration)?,
            options,
            started: Instant::now(),
            route_timeout,
            max_gap,
            failed: false,
            stop_on_drop: true,
        })
    }

    /// Finish a single command while leaving retained subsystem goals in place.
    /// Cancellation or a failed brake still takes the full stop path on drop.
    pub(super) fn finish_command(mut self) -> Result<(), CommandError<R::Error>> {
        self.robot.stop_drive().map_err(CommandError::StopFailed)?;
        self.stop_on_drop = false;
        Ok(())
    }

    pub fn valid_configuration(
        options: DriveOptions,
        period: Duration,
        max_gap: Duration,
        route_timeout: Duration,
    ) -> bool {
        options.valid()
            && CommandClock::new(period).is_some()
            && max_gap >= period
            && route_timeout >= period
    }

    pub(super) fn check_active(&mut self) -> Result<(), CommandError<R::Error>> {
        if self.failed {
            return Err(self.finish_error(CommandError::Cancelled));
        }
        if self.started.elapsed() >= self.route_timeout {
            return Err(self.finish_error(CommandError::RouteTimeout));
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

    /// Remaining time on this runner's original deadline. Helpers and loop
    /// iterations must share this clock rather than starting new runners.
    /// Physical drive options used for this session.
    pub fn options(&self) -> DriveOptions {
        self.options
    }

    pub fn remaining(&self) -> Duration {
        self.route_timeout.saturating_sub(self.started.elapsed())
    }

    /// Wait at zero drive output for a data-dependent condition. The predicate
    /// can capture calibrated thresholds; every tick still services mechanisms,
    /// tracking, cancellation and the original route deadline.
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
    /// a fallback under the original route deadline.
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
        if timeout.is_zero() || !command.valid(self.options) {
            return Err(CommandError::Configuration);
        }
        let mut active = ActiveCommand {
            robot: self.robot,
            completed: false,
        };
        let outcome = async {
            let status = command.status();
            active.robot.show_status(status);
            ozton_runtime::diagnostic_event!("auton_command={status}");
            let started = Instant::now();
            let mut state = State::default();
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
                if self.started.elapsed() >= self.route_timeout {
                    return Err(CommandError::RouteTimeout);
                }
                if dt.is_zero() || dt > self.max_gap {
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
                            + Angle::new::<radian>(if reverse {
                                core::f64::consts::PI
                            } else {
                                0.0
                            });
                        if feedback.pose.position.distance(start.position) > 0.15
                            || wrap_heading(feedback.pose.heading - heading).value.abs() > 1.05
                        {
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
                if step.lost {
                    return Err(CommandError::PathLost);
                }
                if let Err(error) = active.robot.update_subsystems(dt) {
                    if active.robot.recoverable(&error) {
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
                            return Err(CommandError::NoProgress);
                        }
                    }
                } else {
                    progress_pose = None;
                    progress_elapsed = Duration::ZERO;
                }
                // Serial diagnostics can be slower than feedback. Publish the
                // actuator command before offering this sample to a reporter.
                active.robot.log_step(
                    self.started.elapsed(),
                    dt,
                    Length::new::<meter>(step.progress),
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
