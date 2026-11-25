//! Hardware-independent actuator and angular feedback capabilities.

use ozton_math::uom::si::f64::{Angle, AngularVelocity, ElectricPotential, Torque};

use crate::HasBraking;

/// Voltage-controlled actuator with a known output limit.
pub trait VoltageOutput {
    type Error;

    fn max_voltage(&self) -> ElectricPotential;
    fn set_voltage(&mut self, voltage: ElectricPotential) -> Result<(), Self::Error>;
}

/// An actuator that supports both voltage control and explicit stopping.
pub trait MotorOutput: VoltageOutput + HasBraking<BrakeError = <Self as VoltageOutput>::Error> {}

impl<T> MotorOutput for T where
    T: VoltageOutput + HasBraking<BrakeError = <T as VoltageOutput>::Error>
{
}

/// Read accumulated angular position.
pub trait ReadPosition {
    type Error;

    fn position(&self) -> Result<Angle, Self::Error>;
}

/// Assign an angular position reference.
pub trait SetPosition: ReadPosition {
    fn set_position(&mut self, position: Angle) -> Result<(), Self::Error>;
}

/// Read angular velocity.
pub trait ReadVelocity {
    type Error;

    fn velocity(&self) -> Result<AngularVelocity, Self::Error>;
}

/// Read actuator load.
pub trait ReadTorque {
    type Error;

    fn torque(&self) -> Result<Torque, Self::Error>;
}
