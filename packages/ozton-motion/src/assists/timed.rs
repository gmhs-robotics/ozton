//! A bounded assisted arcade segment for an autonomous route.

use std::time::{Duration, Instant};

use ozton_drivetrain::{
    Drivetrain, MotorGroups,
    model::{ArcadeInput, Balancing, Tank},
};
use ozton_runtime::{
    clock::{CONTROL_PERIOD, CommandClock},
    competition::ProgramDrive,
};
use ozton_tracking::sampling::{LocalizationLifecycle, SensorTracking};
use vexide::{
    competition::{self, CompetitionMode},
    smart::PortError,
};

use super::{DriveAssists, MAX_CONTROL_GAP};
use crate::commands::{CommandError, DriveMotionFault};

impl DriveAssists {
    /// Drive with the model's balancing policy for a bounded autonomous segment.
    /// The route supplies fresh normalized inputs each tick, including any custom
    /// scaling. Completion, invalid feedback timing and cancellation stop the drive.
    pub async fn drive_for<M, L>(
        &mut self,
        drive: &mut Drivetrain<M, SensorTracking, L>,
        duration: Duration,
        mut input: impl FnMut() -> ArcadeInput,
    ) -> Result<(), CommandError<DriveMotionFault>>
    where
        M: Tank<Error = PortError> + Balancing<Command = [f64; 2]> + MotorGroups,
        L: LocalizationLifecycle<SensorTracking>,
    {
        self.reset();
        let mut run = ActiveDrive {
            drive,
            assists: self,
            stopped: false,
        };
        let result = async {
            let mut clock = CommandClock::new(CONTROL_PERIOD).ok_or(CommandError::Configuration)?;
            let started = Instant::now();
            let mut dt = Duration::ZERO;
            loop {
                if competition::mode() != CompetitionMode::Autonomous {
                    return Err(CommandError::Cancelled);
                }
                if dt > MAX_CONTROL_GAP {
                    return Err(CommandError::ControlGap);
                }
                if started.elapsed() >= duration {
                    return Ok(());
                }
                let input = input();
                if !input.throttle.is_finite() || !input.turn.is_finite() {
                    return Err(CommandError::Configuration);
                }
                run.assists
                    .drive_balanced(run.drive, input, dt)
                    .map_err(|port| CommandError::Robot(DriveMotionFault::Output(port)))?;
                run.drive
                    .poll_competition_tracking(CompetitionMode::Autonomous);
                dt = clock.tick().await;
            }
        }
        .await;
        match (result, run.stop()) {
            (result, Ok(())) => result,
            (Ok(()), Err(stop)) => Err(CommandError::StopFailed(DriveMotionFault::Brake(stop))),
            (Err(error), Err(stop)) => Err(CommandError::StopFailedAfter {
                cause: Box::new(error),
                stop: DriveMotionFault::Brake(stop),
            }),
        }
    }
}

struct ActiveDrive<'a, M: Tank, L> {
    drive: &'a mut Drivetrain<M, SensorTracking, L>,
    assists: &'a mut DriveAssists,
    stopped: bool,
}

impl<M: Tank, L> ActiveDrive<'_, M, L> {
    fn stop(&mut self) -> Result<(), M::Error> {
        self.assists.reset();
        self.drive.stop_now()?;
        self.stopped = true;
        Ok(())
    }
}

impl<M: Tank, L> Drop for ActiveDrive<'_, M, L> {
    fn drop(&mut self) {
        if !self.stopped {
            let _ = self.stop();
        }
    }
}
