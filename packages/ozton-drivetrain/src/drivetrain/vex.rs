//! VEX motor-group actuation for assembled drivetrains.
use ozton_math::uom::si::f64::{ElectricPotential, Length, Velocity};
use ozton_tracking::{
    HasDriveWheels, TrackingSystem,
    odometric::{SharedMotors, WheelFeedback},
};
use vexide::{
    prelude::Motor,
    smart::{PortError, motor::BrakeMode},
};

use super::Drivetrain;
use crate::{
    Drive, HasBraking, WheelActuation, WheelCommand,
    model::{
        Differential,
        actuation::{WheelSpeedError, coast, command_groups, drive_commands},
    },
};

/// Access the model's motor groups in their fixed wheel order.
pub trait MotorGroups {
    /// Physical wheel-speed commands, one entry per group in model order.
    type WheelSpeeds: AsRef<[Velocity]>;
    /// Voltage commands in the same model order.
    type WheelVoltages: AsRef<[ElectricPotential]>;

    fn motor_groups(&self) -> &[SharedMotors];
    fn motor_groups_mut(&mut self) -> &mut [SharedMotors];
}

impl<M: Drive + MotorGroups<WheelSpeeds = [Velocity; 2]>, T: HasDriveWheels, L>
    Drivetrain<M, T, L>
{
    /// Apply physical wheel velocity targets in model group order.
    /// Conversion to motor RPM uses the same geometry as odometry.
    pub fn drive_wheel_speeds(&mut self, speeds: M::WheelSpeeds) -> Result<(), WheelSpeedError> {
        self.command_wheels(speeds.map(WheelCommand::Velocity))
    }

    /// Apply calibrated velocities and explicit braking modes in group order.
    pub fn command_wheels(&mut self, commands: [WheelCommand; 2]) -> Result<(), WheelSpeedError> {
        let groups = self.model.motor_groups_mut();
        let state = self.state.borrow();
        let result = drive_commands(state.0.calibrated_drive().wheels(), groups, &commands);
        if matches!(result, Err(WheelSpeedError::InvalidCommand)) {
            let _ = coast(groups);
        }
        result
    }
}

impl From<crate::BrakeMode> for BrakeMode {
    fn from(mode: crate::BrakeMode) -> Self {
        match mode {
            crate::BrakeMode::Coast => Self::Coast,
            crate::BrakeMode::Brake => Self::Brake,
            crate::BrakeMode::Hold => Self::Hold,
        }
    }
}

impl HasBraking for Motor {
    type BrakeError = PortError;

    fn apply_brake(&mut self, mode: crate::BrakeMode) -> Result<(), Self::BrakeError> {
        self.brake(mode.into())
    }
}

impl<M, T, L> WheelActuation for Drivetrain<M, T, L>
where
    M: Drive + HasBraking + MotorGroups<WheelSpeeds = [Velocity; 2]>,
    T: HasDriveWheels,
{
    type WheelError = WheelSpeedError;

    fn drive_wheel_commands(
        &mut self,
        commands: [WheelCommand; 2],
    ) -> Result<(), Self::WheelError> {
        self.command_wheels(commands)
    }
}

impl<M: Drive + MotorGroups, T, L> Drivetrain<M, T, L> {
    /// Apply absolute volts to each motor in groups, within their hardware limits.
    pub fn drive_volts(&mut self, volts: M::WheelVoltages) -> Result<(), PortError> {
        let groups = self.model.motor_groups_mut();
        let volts = volts.as_ref();
        if volts.len() != groups.len() || volts.iter().any(|voltage| !voltage.value.is_finite()) {
            return coast(groups);
        }
        command_groups(
            groups.iter_mut().zip(volts.iter().copied()),
            |motor, voltage| {
                if voltage.value == 0.0 {
                    return motor.brake(BrakeMode::Coast);
                }
                let limit = motor.max_voltage();
                motor.set_voltage(voltage.value.clamp(-limit, limit))
            },
        )
    }
}

impl<M: Drive + MotorGroups, T: HasDriveWheels, L> Drivetrain<M, T, L> {
    pub fn sample_feedback(&mut self) -> [Option<WheelFeedback>; 2] {
        self.tracking_mut().calibrated_drive_mut().read_wheels()
    }

    /// Read the last measured translation and velocity without advancing odometry.
    pub fn feedback(&self) -> [Option<WheelFeedback>; 2] {
        self.tracking().calibrated_drive().feedback()
    }
}

impl<G, T: TrackingSystem, L> Drivetrain<Differential<G>, T, L> {
    pub fn measured_voltage(&self) -> [Option<ElectricPotential>; 2] {
        std::array::from_fn(|side| {
            self.model.motor_groups()[side]
                .borrow()
                .first()?
                .voltage()
                .ok()
                .map(ElectricPotential::new::<ozton_math::uom::si::electric_potential::volt>)
        })
    }
}

impl<M: Drive, T: HasDriveWheels, L> Drivetrain<M, T, L> {
    pub fn track_width(&self) -> Length {
        self.tracking().calibrated_drive().track_width()
    }

    pub fn wheel_diameter(&self) -> Length {
        self.tracking().calibrated_drive().wheels().diameter()
    }
}
