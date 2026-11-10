//! Bounded execution for programmed autonomous, including cancellation by drop.

use std::time::{Duration, Instant};

use ozton_math::geometry::{Pose, wrap_heading};
use ozton_runtime::clock::CommandClock;
use vexide::competition::{self, CompetitionMode};

use super::{Alignment, Command, DriveOptions, Feedback, command::State};

/// Connects an application robot to the same executor used by simulated robots.
pub trait AutonRobot {
    type Error;

    /// Capture tracking once. None is a stopped initialization/rebaseline tick.
    fn sample(&mut self) -> Result<Option<Feedback>, Self::Error>;
    fn update_mechanisms(&mut self, dt: Duration) -> Result<(), Self::Error>;
    fn drive(&mut self, meters_per_second: [f64; 2], dt: Duration) -> Result<(), Self::Error>;
    fn stop_drive(&mut self) -> Result<(), Self::Error>;
    /// Stop every output, even when an individual device fails.
    fn stop(&mut self) -> Result<(), Self::Error>;
    fn set_pose(&mut self, pose: Pose) -> Result<(), Self::Error>;
    fn request_distance_reset(&mut self);
    fn log_step(&mut self, _elapsed: Duration, _dt: Duration, _progress: f64, _wheels: [f64; 2]) {}
}

#[derive(Debug)]
pub enum AutonError<E> {
    Configuration,
    Cancelled,
    RouteTimeout,
    CommandTimeout,
    ControlGap,
    InvalidFeedback,
    PathStart,
    PathLost,
    Robot(E),
}

/// Reuse one runner across all commands to enforce an overall route deadline.
/// Dropping it stops drive and mechanisms, including when vexide cancels autonomous.
pub struct Runner<'a, R: AutonRobot> {
    robot: &'a mut R,
    clock: CommandClock,
    options: DriveOptions,
    started: Instant,
    route_timeout: Duration,
    max_gap: Duration,
    failed: bool,
}

impl<'a, R: AutonRobot> Runner<'a, R> {
    pub fn new(
        robot: &'a mut R,
        options: DriveOptions,
        period: Duration,
        max_gap: Duration,
        route_timeout: Duration,
    ) -> Result<Self, AutonError<R::Error>> {
        if !options.valid() || period.is_zero() || max_gap < period || route_timeout < period {
            return Err(AutonError::Configuration);
        }
        Ok(Self {
            robot,
            clock: CommandClock::new(period).ok_or(AutonError::Configuration)?,
            options,
            started: Instant::now(),
            route_timeout,
            max_gap,
            failed: false,
        })
    }

    /// Change retained mechanism goals between commands.
    pub fn robot(&mut self) -> &mut R {
        self.robot
    }

    pub async fn run(
        &mut self,
        command: Command<'_, R>,
        timeout: Duration,
    ) -> Result<(), AutonError<R::Error>> {
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
    ) -> Result<(), AutonError<R::Error>> {
        self.run_with_policy(command, timeout, enabled, false)
            .await
            .map(|_| ())
    }

    /// Returns false after a clean alignment timeout. The next command can run
    /// a fallback under the original route deadline.
    pub async fn try_align(
        &mut self,
        alignment: Alignment,
        timeout: Duration,
    ) -> Result<bool, AutonError<R::Error>> {
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
    ) -> Result<bool, AutonError<R::Error>> {
        self.run_with_policy(Command::Align(alignment), timeout, enabled, true)
            .await
    }

    async fn run_with_policy(
        &mut self,
        command: Command<'_, R>,
        timeout: Duration,
        enabled: impl FnMut() -> bool,
        recover_timeout: bool,
    ) -> Result<bool, AutonError<R::Error>> {
        let result = self
            .execute(command, timeout, enabled, recover_timeout)
            .await;
        if result.is_err() {
            self.failed = true;
            let _ = self.robot.stop();
        }
        result
    }

    async fn execute(
        &mut self,
        mut command: Command<'_, R>,
        timeout: Duration,
        mut enabled: impl FnMut() -> bool,
        recover_timeout: bool,
    ) -> Result<bool, AutonError<R::Error>> {
        if self.failed {
            return Err(AutonError::Cancelled);
        }
        if timeout.is_zero() || !command.valid(self.options) {
            return Err(AutonError::Configuration);
        }
        let mut active = ActiveCommand {
            robot: self.robot,
            completed: false,
        };
        let started = Instant::now();
        let mut state = State::default();
        let mut initialized = false;
        loop {
            let dt = self.clock.tick().await;
            let elapsed = started.elapsed();
            if !enabled() {
                return Err(AutonError::Cancelled);
            }
            if self.started.elapsed() >= self.route_timeout {
                return Err(AutonError::RouteTimeout);
            }
            if dt.is_zero() || dt > self.max_gap {
                return Err(AutonError::ControlGap);
            }
            if elapsed >= timeout {
                if recover_timeout {
                    active.robot.stop_drive().map_err(AutonError::Robot)?;
                    active.completed = true;
                    return Ok(false);
                }
                return Err(AutonError::CommandTimeout);
            }
            if matches!(command, Command::DistanceReset) {
                active.robot.request_distance_reset();
            }
            let feedback = active.robot.sample().map_err(AutonError::Robot)?;
            let Some(feedback) = feedback else {
                active.robot.stop_drive().map_err(AutonError::Robot)?;
                continue;
            };
            if !feedback.pose.is_finite()
                || ![
                    feedback.forward_travel,
                    feedback.forward_velocity,
                    feedback.angular_velocity,
                ]
                .iter()
                .all(|value| value.is_finite())
            {
                return Err(AutonError::InvalidFeedback);
            }
            if !initialized {
                if let Some((path, reverse)) = command.path_start() {
                    let start = path.start();
                    let heading = start.heading + if reverse { core::f64::consts::PI } else { 0.0 };
                    if feedback.pose.position.distance(start.position) > 0.15
                        || wrap_heading(feedback.pose.heading - heading).abs() > 1.05
                    {
                        return Err(AutonError::PathStart);
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
                return Err(AutonError::PathLost);
            }
            active
                .robot
                .update_mechanisms(dt)
                .map_err(AutonError::Robot)?;
            active
                .robot
                .log_step(self.started.elapsed(), dt, step.progress, step.wheels);
            if step.complete {
                active.robot.stop_drive().map_err(AutonError::Robot)?;
                if let Command::Contact(contact) = command {
                    active
                        .robot
                        .set_pose(contact.known_pose)
                        .map_err(AutonError::Robot)?;
                }
                active.completed = true;
                return Ok(true);
            }
            if step.wheels == [0.0; 2] {
                active.robot.stop_drive().map_err(AutonError::Robot)?;
            } else {
                active
                    .robot
                    .drive(step.wheels, dt)
                    .map_err(AutonError::Robot)?;
            }
        }
    }
}

struct ActiveCommand<'a, R: AutonRobot> {
    robot: &'a mut R,
    completed: bool,
}

impl<R: AutonRobot> Drop for ActiveCommand<'_, R> {
    fn drop(&mut self) {
        if !self.completed {
            let _ = self.robot.stop();
        }
    }
}

impl<R: AutonRobot> Drop for Runner<'_, R> {
    fn drop(&mut self) {
        let _ = self.robot.stop();
    }
}
