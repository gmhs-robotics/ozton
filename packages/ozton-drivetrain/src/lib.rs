//! A motion model, the robot's physical geometry, and its tracking system.
use model::{Balancing, Differential, DrivetrainModel, Holonomic, Tank};
use ozton_math::uom::si::f64::{AngularVelocity, ElectricPotential, Length, Velocity};
pub use ozton_tracking::encoders::{DriveConfigError, WheelFeedback, WheelGearing, WheelLoad};
use ozton_tracking::{
    Tracking,
    encoders::{DriveEncoders, Wheels, positive},
};
use vexide::{
    prelude::Motor,
    smart::{PortError, motor::BrakeMode},
};
pub use wheels::WheelSpeedError;

pub mod model;
mod wheels;

/// Access the model's motor groups in their fixed wheel order.
pub trait MotorGroups {
    /// Physical wheel-speed commands, one entry per group in model order.
    type WheelSpeeds: AsRef<[Velocity]>;

    fn motor_groups(&self) -> &[Box<[Motor]>];
    fn motor_groups_mut(&mut self) -> &mut [Box<[Motor]>];
}

/// A drivetrain owns geometry and feedback calibration independently of how it mixes motor outputs.
pub struct Drivetrain<M: DrivetrainModel, T: Tracking, L> {
    pub model: M,
    pub tracking: T,
    pub localization: L,
    wheels: Option<Wheels>,
    track_width: Option<Length>,
    wheelbase: Option<Length>,
    driver_output: bool,
    wheel_feedback: [Option<WheelFeedback>; 2],
}

impl<M: DrivetrainModel, T: Tracking, L> Drivetrain<M, T, L> {
    pub fn new(model: M, tracking: T, localization: L) -> Self {
        Self {
            model,
            tracking,
            localization,
            wheels: None,
            track_width: None,
            wheelbase: None,
            driver_output: false,
            wheel_feedback: [None; 2],
        }
    }

    /// Consume the write marker used by competition output watchdogs.
    pub fn take_output(&mut self) -> bool {
        std::mem::take(&mut self.driver_output)
    }

    pub fn with_track_width(mut self, width: Length) -> Result<Self, DriveConfigError> {
        positive(width.value, DriveConfigError::TrackWidth)?;
        self.track_width = Some(width);
        Ok(self)
    }

    pub fn with_wheelbase(mut self, length: Length) -> Result<Self, DriveConfigError> {
        positive(length.value, DriveConfigError::Wheelbase)?;
        self.wheelbase = Some(length);
        Ok(self)
    }

    pub fn track_width(&self) -> Option<Length> {
        self.track_width
    }
    pub fn wheelbase(&self) -> Option<Length> {
        self.wheelbase
    }
    pub fn wheel_diameter(&self) -> Option<Length> {
        self.wheels.as_ref().map(Wheels::diameter)
    }
}

impl<M: DrivetrainModel + MotorGroups, T: Tracking, L> Drivetrain<M, T, L> {
    /// Set diameter and one ratio per motor, in model group and motor order.
    pub fn with_wheel_geometry(
        mut self,
        diameter: Length,
        gearing: &[&[WheelGearing]],
    ) -> Result<Self, DriveConfigError> {
        self.wheels = Some(Wheels::geared(
            diameter,
            self.model.motor_groups(),
            gearing,
        )?);
        Ok(self)
    }

    /// Infer ratios from the measured common free wheel RPM and each motor cartridge.
    pub fn with_wheel_feedback(
        mut self,
        diameter: Length,
        free_speed: AngularVelocity,
    ) -> Result<Self, DriveConfigError> {
        self.wheels = Some(Wheels::inferred(
            diameter,
            self.model.motor_groups(),
            free_speed,
        )?);
        Ok(self)
    }

    /// Apply physical wheel velocity targets in model group order.
    /// Conversion to motor RPM uses the same geometry as odometry.
    pub fn drive_wheel_speeds<const N: usize>(
        &mut self,
        speeds: [Velocity; N],
    ) -> Result<(), WheelSpeedError> {
        self.set_wheel_speeds(&speeds)
    }

    /// Apply a slice of physical wheel speeds in model group order.
    pub fn set_wheel_speeds(&mut self, speeds: &[Velocity]) -> Result<(), WheelSpeedError> {
        let groups = self.model.motor_groups_mut();
        let result = self
            .wheels
            .as_ref()
            .ok_or(WheelSpeedError::MissingWheelGeometry)
            .and_then(|wheels| wheels::drive_speed(wheels, groups, speeds));
        if matches!(
            result,
            Err(WheelSpeedError::InvalidCommand | WheelSpeedError::MissingWheelGeometry)
        ) {
            let _ = coast(groups);
        }
        result
    }

    pub fn wheel_load<const N: usize>(&self) -> [Option<WheelLoad>; N] {
        std::array::from_fn(|index| {
            self.wheels
                .as_ref()?
                .load(index, self.model.motor_groups().get(index)?)
        })
    }

    /// Apply absolute volts to each motor in groups, within their hardware limits.
    pub fn drive_volts<const N: usize>(
        &mut self,
        volts: [ElectricPotential; N],
    ) -> Result<(), PortError> {
        let groups = self.model.motor_groups_mut();
        if N != groups.len() || volts.iter().any(|voltage| !voltage.value.is_finite()) {
            return coast(groups);
        }
        groups
            .iter_mut()
            .zip(volts)
            .fold(Ok(()), |result, (group, voltage)| {
                result.and(each_motor(group, |motor| {
                    if voltage.value == 0.0 {
                        return motor.brake(BrakeMode::Coast);
                    }
                    let limit = motor.max_voltage();
                    motor.set_voltage(voltage.value.clamp(-limit, limit))
                }))
            })
    }
}

impl<M: DrivetrainModel + MotorGroups, T: Tracking, L> Drivetrain<M, T, L> {
    pub fn sample_feedback(&mut self) -> [Option<WheelFeedback>; 2] {
        self.wheel_feedback = self
            .wheels
            .as_mut()
            .map_or([None; 2], |wheels| wheels.sample(self.model.motor_groups()));
        self.wheel_feedback
    }

    /// Read the last measured translation and velocity without advancing odometry.
    pub fn feedback(&self) -> [Option<WheelFeedback>; 2] {
        self.wheel_feedback
    }
}

impl<T: Tracking, L> Drivetrain<Differential, T, L> {
    pub fn measured_voltage(&self) -> [Option<ElectricPotential>; 2] {
        std::array::from_fn(|side| {
            self.model.motor_groups()[side]
                .first()?
                .voltage()
                .ok()
                .map(ElectricPotential::new::<ozton_math::uom::si::electric_potential::volt>)
        })
    }
}

impl<T: Tracking, L> DriveEncoders for Drivetrain<Differential, T, L> {
    fn track_width(&self) -> Option<Length> {
        self.track_width
    }
    fn tracking_positions(&mut self) -> [Option<Length>; 2] {
        self.sample_feedback()
            .map(|sample| sample.map(|sample| sample.position))
    }
}

impl<M: DrivetrainModel, T: Tracking, L> DrivetrainModel for Drivetrain<M, T, L> {
    type Error = M::Error;
}

impl<M: Tank, T: Tracking, L> Tank for Drivetrain<M, T, L> {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), Self::Error> {
        self.driver_output = true;
        self.model.drive_tank(left, right)
    }

    fn stop_now(&mut self) -> Result<(), Self::Error> {
        self.model.stop_now()
    }
}

impl<M: Holonomic, T: Tracking, L> Holonomic for Drivetrain<M, T, L> {
    fn drive_vector(&mut self, vector: glam::DVec2, turn: f64) -> Result<(), Self::Error> {
        self.driver_output = true;
        self.model.drive_vector(vector, turn)
    }
}

impl<M: Balancing, T: Tracking, L> Balancing for Drivetrain<M, T, L> {
    type Command = M::Command;

    fn drive_balanced(&mut self, command: Self::Command) -> Result<(), Self::Error> {
        self.driver_output = true;
        self.model.drive_balanced(command)
    }
}

pub(crate) fn normalized_voltage<const N: usize>(
    groups: &mut [Box<[Motor]>; N],
    powers: [f64; N],
) -> Result<(), PortError> {
    if powers.iter().any(|power| !power.is_finite()) {
        return coast(groups);
    }
    groups
        .iter_mut()
        .zip(ozton_math::desaturate(powers, 1.0))
        .fold(Ok(()), |result, (group, power)| {
            result.and(each_motor(group, |motor| {
                if power == 0.0 {
                    motor.brake(BrakeMode::Coast)
                } else {
                    motor.set_voltage(power * motor.max_voltage())
                }
            }))
        })
}

pub(crate) fn each_motor(
    motors: &mut [Motor],
    mut command: impl FnMut(&mut Motor) -> Result<(), PortError>,
) -> Result<(), PortError> {
    motors
        .iter_mut()
        .fold(Ok(()), |result, motor| result.and(command(motor)))
}

pub(crate) fn coast(groups: &mut [Box<[Motor]>]) -> Result<(), PortError> {
    groups.iter_mut().fold(Ok(()), |result, group| {
        result.and(each_motor(group, |motor| motor.brake(BrakeMode::Coast)))
    })
}
