//! Shared actuator and position feedback interfaces for mechanisms.

use ozton_math::uom::si::{
    angular_velocity::radian_per_second,
    electric_potential::volt,
    f64::{Angle, AngularVelocity, ElectricPotential, Torque},
    torque::newton_meter,
};
use vexide::{
    math::Angle as VexAngle,
    prelude::{Motor, RotationSensor},
    smart::{PortError, motor::BrakeMode},
};

/// Motor operations used by the mechanism controller, also implemented by fault-test devices.
pub trait MotorDevice {
    fn position(&self) -> Result<Angle, PortError>;
    fn set_position(&mut self, position: Angle) -> Result<(), PortError>;
    fn brake(&mut self, mode: BrakeMode) -> Result<(), PortError>;
    fn max_voltage(&self) -> ElectricPotential;
    fn set_voltage(&mut self, voltage: ElectricPotential) -> Result<(), PortError>;
}

impl MotorDevice for Motor {
    fn position(&self) -> Result<Angle, PortError> {
        Motor::position(self).map(|position| ozton_math::qty!((position.as_radians()) rad))
    }
    fn set_position(&mut self, position: Angle) -> Result<(), PortError> {
        Motor::set_position(self, VexAngle::from_radians(position.value))
    }
    fn brake(&mut self, mode: BrakeMode) -> Result<(), PortError> {
        Motor::brake(self, mode)
    }
    fn max_voltage(&self) -> ElectricPotential {
        ElectricPotential::new::<volt>(Motor::max_voltage(self))
    }
    fn set_voltage(&mut self, voltage: ElectricPotential) -> Result<(), PortError> {
        Motor::set_voltage(self, voltage.value)
    }
}

/// Accumulated angular position and reference assignment.
pub trait PositionSensor {
    fn position(&self) -> Result<Angle, PortError>;
    fn set_position(&mut self, position: Angle) -> Result<(), PortError>;
}

impl PositionSensor for RotationSensor {
    fn position(&self) -> Result<Angle, PortError> {
        RotationSensor::position(self).map(|position| ozton_math::qty!((position.as_radians()) rad))
    }
    fn set_position(&mut self, position: Angle) -> Result<(), PortError> {
        RotationSensor::set_position(self, VexAngle::from_radians(position.value))
    }
}

/// Motors with integrated velocity feedback as an angular velocity.
pub trait VelocityMotor: MotorDevice {
    fn velocity(&self) -> Result<AngularVelocity, PortError>;
}

impl VelocityMotor for Motor {
    fn velocity(&self) -> Result<AngularVelocity, PortError> {
        Motor::velocity(self).map(|rpm| {
            AngularVelocity::new::<radian_per_second>(rpm * (core::f64::consts::TAU / 60.0))
        })
    }
}

/// Optional load feedback in newton-metres. Driver control need not read it.
pub trait TorqueMotor: MotorDevice {
    fn torque(&self) -> Result<Torque, PortError>;
}

impl TorqueMotor for Motor {
    fn torque(&self) -> Result<Torque, PortError> {
        Motor::torque(self).map(Torque::new::<newton_meter>)
    }
}
