//! Wheel actuation and feedback hooks used by competition dispatch.

use std::time::Duration;

use ozton_drivetrain::{Drivetrain, HasLocalization, MotorGroups, WheelSpeedError, model::Drive};
use ozton_tracking::{TrackingSystem, sampling::LocalizationLifecycle};
use vexide::{competition::CompetitionMode, smart::PortError};

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

    fn competition_mode_changed(&mut self, _mode: vexide::competition::CompetitionMode) {}

    fn poll_competition_tracking(&mut self, _mode: vexide::competition::CompetitionMode) {}

    /// Consume the marker set by a direct driver write through the drivetrain.
    fn take_driver_output(&mut self) -> bool {
        false
    }
}

impl<M, T, L> ProgramDrive for Drivetrain<M, T, L>
where
    M: Drive<Error = PortError> + MotorGroups,
    T: TrackingSystem,
    L: LocalizationLifecycle<T>,
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
        self.set_wheel_speeds(speeds.as_ref())
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.localization.cancel_pending();
        self.stop_now().map_err(WheelSpeedError::Port)
    }

    fn competition_mode_changed(&mut self, mode: CompetitionMode) {
        self.localization.mode_changed(mode);
    }

    fn poll_competition_tracking(&mut self, mode: CompetitionMode) {
        if mode != CompetitionMode::Disabled {
            self.refresh_localization();
        }
    }

    fn take_driver_output(&mut self) -> bool {
        self.take_output()
    }
}
