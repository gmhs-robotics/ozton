//! A bounded assisted arcade segment for an autonomous route.

use std::time::{Duration, Instant};

use ozton_drivetrain::{
    Drivetrain, MotorGroups,
    assist::{AssistTracking, AssistedDrivetrain, DriveAssist},
    model::{ArcadeInput, Tank},
};
use ozton_runtime::{
    clock::{CONTROL_PERIOD, CommandClock},
    competition::ProgramDrive,
};
use ozton_tracking::sampling::LocalizationLifecycle;
use vexide::{
    competition::{self, CompetitionMode},
    smart::PortError,
};

const MAX_CONTROL_GAP: Duration = Duration::from_millis(100);
use crate::commands::{CommandError, DriveMotionFault};

/// Bounded assisted commands for autonomous routes.
#[allow(async_fn_in_trait)]
pub trait AssistedMotion {
    async fn drive_for(
        self,
        duration: Duration,
        input: impl FnMut() -> ArcadeInput,
    ) -> Result<(), CommandError<DriveMotionFault>>;
}

impl<M, T, L, A> AssistedMotion for AssistedDrivetrain<'_, M, T, L, A>
where
    M: Tank<Error = PortError> + MotorGroups,
    T: AssistTracking,
    L: LocalizationLifecycle<T>,
    A: DriveAssist<M>,
{
    /// Run an assisted autonomous segment. Stops on exit, including cancellation.
    async fn drive_for(
        self,
        duration: Duration,
        mut input: impl FnMut() -> ArcadeInput,
    ) -> Result<(), CommandError<DriveMotionFault>> {
        let drive = self.into_inner();
        drive.reset_assists();
        let mut run = ActiveDrive {
            drive,
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
                run.drive
                    .assisted()
                    .try_drive(input)
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

struct ActiveDrive<'a, M: Tank, T: AssistTracking, L, A: DriveAssist<M>> {
    drive: &'a mut Drivetrain<M, T, L, A>,
    stopped: bool,
}

impl<M: Tank, T: AssistTracking, L, A: DriveAssist<M>> ActiveDrive<'_, M, T, L, A> {
    fn stop(&mut self) -> Result<(), M::Error> {
        self.drive.stop_now()?;
        self.stopped = true;
        Ok(())
    }
}

impl<M: Tank, T: AssistTracking, L, A: DriveAssist<M>> Drop for ActiveDrive<'_, M, T, L, A> {
    fn drop(&mut self) {
        if !self.stopped {
            let _ = self.stop();
        }
    }
}
