//! Timed autonomous stages independent of drivetrain kinematics.

use std::time::{Duration, Instant};

use ozton_drivetrain::model::ProgramDrive;
use ozton_runtime::{
    CompetitionBot, RobotLayout,
    clock::CommandClock,
    competition::{CompetitionRobot, Program, Tick},
};
use vexide::competition::{self, CompetitionMode};

use super::AutonError;

type Error<P> = <P as Program<CompetitionBot<P>, <P as RobotLayout>::Drivetrain>>::Error;

struct ActiveStage<'a, R: CompetitionRobot> {
    robot: &'a mut R,
    running: bool,
}

impl<R: CompetitionRobot> Drop for ActiveStage<'_, R> {
    fn drop(&mut self) {
        if self.running {
            self.robot.set_autonomous_start(None);
            let _ = self.robot.stop();
        }
    }
}

/// Timed commands and mechanism waits on the installed program. These stages
/// share the current route's deadline, including across separate handles.
pub struct Auton<'a, P: RobotLayout> {
    robot: &'a mut CompetitionBot<P>,
}

impl<'a, P> Auton<'a, P>
where
    P: RobotLayout + Program<CompetitionBot<P>, P::Drivetrain>,
    P::Drivetrain: ProgramDrive,
    CompetitionBot<P>: CompetitionRobot<Error = Error<P>>,
{
    pub fn new(robot: &'a mut CompetitionBot<P>) -> Self {
        Self { robot }
    }

    pub fn mechanisms(&mut self) -> &mut P::Mechanisms {
        &mut self.robot.mechanisms
    }

    /// Keep servicing mechanism goals with the drivetrain stopped.
    pub async fn wait(&mut self, duration: Duration) -> Result<(), AutonError<Error<P>>> {
        self.stage(None, duration, |_| false, true).await
    }

    /// Keep servicing mechanism goals until a measured condition is met.
    pub async fn wait_until(
        &mut self,
        timeout: Duration,
        ready: impl FnMut(&P::Mechanisms) -> bool,
    ) -> Result<(), AutonError<Error<P>>> {
        self.stage(None, timeout, ready, false).await
    }

    /// Run a drivetrain-specific command under the programmed route clock.
    pub async fn drive_for(
        &mut self,
        command: <P::Drivetrain as ProgramDrive>::AutonCommand,
        duration: Duration,
    ) -> Result<(), AutonError<Error<P>>>
    where
        <P::Drivetrain as ProgramDrive>::AutonCommand: Clone,
    {
        self.stage(Some(&|| command.clone()), duration, |_| false, true)
            .await
    }

    async fn stage(
        &mut self,
        command: Option<&dyn Fn() -> <P::Drivetrain as ProgramDrive>::AutonCommand>,
        duration: Duration,
        mut ready: impl FnMut(&P::Mechanisms) -> bool,
        duration_is_success: bool,
    ) -> Result<(), AutonError<Error<P>>> {
        let mut active = ActiveStage {
            robot: &mut *self.robot,
            running: true,
        };
        let result = Self::run_stage(
            active.robot,
            command,
            duration,
            &mut ready,
            duration_is_success,
        )
        .await;
        active.running = false;
        let robot = &mut *active.robot;
        if let Err(error) = result {
            robot.autonomous.cancel_motion();
            return match robot.stop() {
                Ok(()) => Err(error),
                Err(stop) => Err(AutonError::StopFailedAfter {
                    cause: Box::new(error),
                    stop,
                }),
            };
        }
        if let Err(error) = robot.drivetrain.stop() {
            robot.autonomous.cancel_motion();
            let _ = robot.stop();
            return Err(AutonError::StopFailed(P::drive_error(error, true)));
        }
        Ok(())
    }

    async fn run_stage(
        robot: &mut CompetitionBot<P>,
        command: Option<&dyn Fn() -> <P::Drivetrain as ProgramDrive>::AutonCommand>,
        duration: Duration,
        ready: &mut impl FnMut(&P::Mechanisms) -> bool,
        duration_is_success: bool,
    ) -> Result<(), AutonError<Error<P>>> {
        let Some(route_start) = robot.autonomous.started_at() else {
            return Err(AutonError::Cancelled);
        };
        let period = P::CONTROL_PERIOD;
        let mut clock = CommandClock::new(period).ok_or(AutonError::Configuration)?;
        let started = Instant::now();
        let mut dt = period;
        if command.is_none() {
            robot
                .drivetrain
                .stop()
                .map_err(|e| AutonError::StopFailed(P::drive_error(e, true)))?;
        }
        loop {
            if competition::mode() != CompetitionMode::Autonomous {
                return Err(AutonError::Cancelled);
            }
            if route_start.elapsed() >= P::AUTON_TIMEOUT {
                return Err(AutonError::RouteTimeout);
            }
            if dt > P::MAX_CONTROL_GAP {
                return Err(AutonError::ControlGap);
            }
            if ready(&robot.mechanisms) {
                return Ok(());
            }
            if started.elapsed() >= duration {
                return if duration_is_success {
                    Ok(())
                } else {
                    Err(AutonError::CommandTimeout)
                };
            }
            let tick = Tick::start(dt);
            P::update_mechanisms(robot, CompetitionMode::Autonomous, dt)
                .map_err(AutonError::Robot)?;
            if let Some(command) = command {
                robot
                    .drivetrain
                    .apply_auton(command(), dt)
                    .map_err(|error| AutonError::Robot(P::drive_error(error, false)))?;
            }
            robot.after_tick(CompetitionMode::Autonomous, tick);
            dt = clock.tick().await;
        }
    }
}
