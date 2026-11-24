//! Wheel actuation and feedback hooks used by competition dispatch.

use std::time::Duration;

use ozton_drivetrain::{Drivetrain, MotorGroups, WheelSpeedError, model::Drive};
use ozton_tracking::TrackingSystem;
use vexide::smart::PortError;

/// Commands consumed by a drivetrain during an Ozton competition.
pub trait ProgramDrive {
    type Error;
    type DriverCommand;
    type AutonCommand;

    fn apply_driver(
        &mut self,
        command: Self::DriverCommand,
        dt: std::time::Duration,
    ) -> Result<(), Self::Error>;
    fn apply_auton(
        &mut self,
        command: Self::AutonCommand,
        dt: std::time::Duration,
    ) -> Result<(), Self::Error>;
    fn stop(&mut self) -> Result<(), Self::Error>;

    /// Consume the marker set by a direct driver write through the drivetrain.
    fn take_driver_output(&mut self) -> bool {
        false
    }
}

impl<M, T, L> ProgramDrive for Drivetrain<M, T, L>
where
    M: Drive<Error = PortError> + MotorGroups,
    T: TrackingSystem,
{
    type Error = WheelSpeedError;
    type DriverCommand = (ozton_math::geometry::DVec2, f64);
    type AutonCommand = M::WheelSpeeds;

    fn apply_driver(
        &mut self,
        (translation, turn): Self::DriverCommand,
        _: Duration,
    ) -> Result<(), Self::Error> {
        self.drive_vector(translation, turn)
            .map_err(WheelSpeedError::Port)
    }

    fn apply_auton(&mut self, speeds: Self::AutonCommand, _: Duration) -> Result<(), Self::Error> {
        self.drive_wheel_speeds(speeds)
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.stop_now().map_err(WheelSpeedError::Port)
    }

    fn take_driver_output(&mut self) -> bool {
        self.take_output()
    }
}
