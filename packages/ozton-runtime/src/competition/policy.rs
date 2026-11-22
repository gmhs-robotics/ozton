//! Program policy and mechanism hooks used by competition dispatch.
use super::*;

/// A mechanism or mechanism group serviced by the competition runtime.
pub trait Mechanism<E> {
    fn update(&mut self, _mode: CompetitionMode, _dt: Duration) -> Result<(), E> {
        Ok(())
    }

    fn stop(&mut self) -> Result<(), E> {
        Ok(())
    }

    fn finish_driver_route(&mut self) -> Result<(), E> {
        self.stop()
    }
}

impl<E> Mechanism<E> for () {}

/// Low-level competition hooks for applications that provide their own policy.
/// Use [`CompetitionLayout`] to register standalone driver and route callbacks.
#[allow(async_fn_in_trait)]
pub trait Program<R, D: ProgramDrive> {
    type Error: std::fmt::Display;
    const CONTROL_PERIOD: Duration = CONTROL_PERIOD;
    const AUTON_TIMEOUT: Duration = Duration::from_secs(15);
    const MAX_CONTROL_GAP: Duration = Duration::from_millis(100);

    /// Translate a drive failure into the program's error type.
    fn drive_error(error: D::Error, stopping: bool) -> Self::Error;

    /// Driver callback. Read the controller directly. Return an error to stop
    /// the robot, for example if the controller is unavailable.
    async fn driver(robot: &mut R) -> Result<(), Self::Error>;

    /// Refresh lightweight inputs needed by a driver callback.
    fn before_driver(_robot: &mut R) {}

    /// Bit mask of beams blocked by the current mechanism pose.
    fn range_occlusion(_robot: &R) -> u8 {
        0
    }

    /// Service the same mechanism coordinator during driver and autonomous ticks.
    fn update_mechanisms(
        _robot: &mut R,
        _mode: CompetitionMode,
        _dt: Duration,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Stop custom actuators and clear pending requests. Ozton stops the drive separately.
    fn stop_mechanisms(_robot: &mut R) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Remove actuator output for a recoverable motion failure.
    fn pause_mechanisms(robot: &mut R) -> Result<(), Self::Error> {
        Self::stop_mechanisms(robot)
    }

    /// Release a finished driver's mechanism commands. The drive remains under joystick control.
    fn finish_driver_route(robot: &mut R) -> Result<(), Self::Error> {
        Self::stop_mechanisms(robot)
    }

    /// Optional tracking work runs after driver outputs and within the tick budget.
    fn poll_tracking(_robot: &mut R, _mode: CompetitionMode) {}
    fn mode_changed(_robot: &mut R, _mode: CompetitionMode) {}
    fn after_tick(_robot: &mut R, _mode: CompetitionMode, _tick: Tick) {}
    fn route_error(
        _robot: &mut R,
        _mode: CompetitionMode,
        _name: &'static str,
        _error: &Self::Error,
    ) -> Option<&'static str> {
        None
    }
}

/// An application layout with registered driver and autonomous callbacks.
pub trait CompetitionLayout: crate::RobotLayout {
    type Error: std::fmt::Display;
    const CONTROL_PERIOD: Duration = CONTROL_PERIOD;
}

impl<L> Program<crate::CompetitionBot<L>, L::Drivetrain> for L
where
    L: CompetitionLayout,
    L::Drivetrain: ProgramDrive,
    L::Error: From<<L::Drivetrain as ProgramDrive>::Error>,
    L::Routes: Routes<crate::CompetitionBot<L>, L::Error>,
{
    type Error = L::Error;
    const CONTROL_PERIOD: Duration = L::CONTROL_PERIOD;

    fn drive_error(error: <L::Drivetrain as ProgramDrive>::Error, _: bool) -> Self::Error {
        error.into()
    }

    async fn driver(robot: &mut crate::CompetitionBot<L>) -> Result<(), Self::Error> {
        if let Some(driver) = robot.autonomous.driver {
            driver.run(robot).await
        } else {
            Ok(())
        }
    }
}
