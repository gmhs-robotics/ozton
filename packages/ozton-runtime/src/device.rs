//! Shared actuator and position feedback interfaces for mechanisms.

use vexide::{
    math::Angle,
    prelude::{Motor, RotationSensor},
    smart::{PortError, motor::BrakeMode},
};

/// Motor operations used by the mechanism controller, also implemented by fault-test devices.
pub trait MotorDevice {
    fn position(&self) -> Result<Angle, PortError>;
    fn set_position(&mut self, position: Angle) -> Result<(), PortError>;
    fn brake(&mut self, mode: BrakeMode) -> Result<(), PortError>;
    fn max_voltage(&self) -> f64;
    /// Voltage, amperes, Celsius, RPM, and newton-metres; unavailable samples are NaN.
    #[cfg(debug_assertions)]
    fn debug_measurements(&self) -> (f64, f64, f64, f64, f64) {
        (f64::NAN, f64::NAN, f64::NAN, f64::NAN, f64::NAN)
    }
    fn set_voltage(&mut self, voltage: f64) -> Result<(), PortError>;
}

impl MotorDevice for Motor {
    fn position(&self) -> Result<Angle, PortError> {
        Motor::position(self)
    }
    fn set_position(&mut self, position: Angle) -> Result<(), PortError> {
        Motor::set_position(self, position)
    }
    fn brake(&mut self, mode: BrakeMode) -> Result<(), PortError> {
        Motor::brake(self, mode)
    }
    fn max_voltage(&self) -> f64 {
        Motor::max_voltage(self)
    }
    #[cfg(debug_assertions)]
    fn debug_measurements(&self) -> (f64, f64, f64, f64, f64) {
        (
            self.voltage().unwrap_or(f64::NAN),
            self.current().unwrap_or(f64::NAN),
            self.temperature().unwrap_or(f64::NAN),
            self.velocity().unwrap_or(f64::NAN),
            self.torque().unwrap_or(f64::NAN),
        )
    }
    fn set_voltage(&mut self, voltage: f64) -> Result<(), PortError> {
        Motor::set_voltage(self, voltage)
    }
}

/// Accumulated angular feedback; adapters retain device units as `Angle`.
pub trait PositionSensor {
    fn position(&self) -> Result<Angle, PortError>;
    fn set_position(&mut self, position: Angle) -> Result<(), PortError>;
}

impl PositionSensor for RotationSensor {
    fn position(&self) -> Result<Angle, PortError> {
        RotationSensor::position(self)
    }
    fn set_position(&mut self, position: Angle) -> Result<(), PortError> {
        RotationSensor::set_position(self, position)
    }
}

/// Motors with integrated velocity feedback, expressed in revolutions per minute.
pub trait VelocityMotor: MotorDevice {
    fn velocity(&self) -> Result<f64, PortError>;
}

impl VelocityMotor for Motor {
    fn velocity(&self) -> Result<f64, PortError> {
        Motor::velocity(self)
    }
}

/// Optional load feedback in newton-metres. Driver control need not read it.
pub trait TorqueMotor: MotorDevice {
    fn torque(&self) -> Result<f64, PortError>;
}

impl TorqueMotor for Motor {
    fn torque(&self) -> Result<f64, PortError> {
        Motor::torque(self)
    }
}
