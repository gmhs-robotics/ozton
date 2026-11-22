//! Wheel actuation and feedback hooks used by competition dispatch.

use std::time::Duration;

use ozton_drivetrain::{
    Drivetrain, MotorGroups, WheelSpeedError,
    assist::DriveAssist,
    model::{ArcadeInput, Tank},
};
use ozton_tracking::{Tracking, sampling::LocalizationLifecycle};
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
    fn take_driver_fault(&mut self) -> Option<Self::Error> {
        None
    }

    fn take_driver_output(&mut self) -> bool {
        false
    }
}

impl<M, T, L, A> ProgramDrive for Drivetrain<M, T, L, A>
where
    M: Tank<Error = PortError> + MotorGroups,
    T: Tracking,
    L: LocalizationLifecycle<T>,
    A: DriveAssist<M>,
{
    type Error = WheelSpeedError;
    type DriverCommand = ArcadeInput;
    type AutonCommand = M::WheelSpeeds;

    fn apply_driver(&mut self, command: ArcadeInput, _: Duration) -> Result<(), Self::Error> {
        use ozton_drivetrain::model::Arcade;
        let (throttle, turn) = command.scaled_axes();
        self.drive_arcade(throttle, turn)
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
        self.reset_assists();
        self.localization.mode_changed(mode);
    }

    fn poll_competition_tracking(&mut self, mode: CompetitionMode) {
        if mode == CompetitionMode::Disabled {
            return;
        }
        let groups = self.model.motor_groups();
        let positions =
            if groups.len() == 2 && self.localization.uses_drive_encoders(&self.tracking) {
                self.sample_feedback()
                    .map(|sample| sample.map(|sample| sample.position))
            } else {
                [None; 2]
            };
        let width = self.track_width();
        self.localization.poll(&mut self.tracking, positions, width);
    }

    fn take_driver_fault(&mut self) -> Option<Self::Error> {
        self.take_output_fault().map(WheelSpeedError::Port)
    }

    fn take_driver_output(&mut self) -> bool {
        self.take_output()
    }
}
