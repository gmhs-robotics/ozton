//! Vexide actuator adapter for a reusable Ozton control axis.

use std::time::Duration;

use ozton_control::axis::{AxisController, AxisSample, Target};
use vexide::{
    prelude::Motor,
    smart::{PortError, motor::BrakeMode},
};

#[derive(Debug)]
pub enum MotorError {
    Port(PortError),
    InvalidFeedback,
}

/// Controls a motor in output radians and radians per second.
pub struct PidMotor {
    pub motor: Motor,
    controller: AxisController,
    output_per_motor: f64,
    brake: BrakeMode,
}

impl PidMotor {
    pub fn new(
        motor: Motor,
        controller: AxisController,
        output_per_motor: f64,
        brake: BrakeMode,
    ) -> Option<Self> {
        if !output_per_motor.is_finite() || output_per_motor <= 0.0 {
            return None;
        }
        Some(Self {
            motor,
            controller,
            output_per_motor,
            brake,
        })
    }

    /// Use this with either the integrated encoder or an independently sampled joint encoder.
    pub fn apply(
        &mut self,
        sample: AxisSample,
        target: Target,
        dt: Duration,
    ) -> Result<f64, MotorError> {
        let Some(volts) = self.controller.update(sample, target, dt) else {
            self.stop().map_err(MotorError::Port)?;
            return Err(MotorError::InvalidFeedback);
        };
        let voltage = volts.clamp(-self.motor.max_voltage(), self.motor.max_voltage());
        if let Err(error) = self.motor.set_voltage(voltage) {
            let _ = self.stop();
            return Err(MotorError::Port(error));
        }
        Ok(voltage)
    }

    pub fn update(&mut self, target: Target, dt: Duration) -> Result<f64, MotorError> {
        let sample = self.motor.position().and_then(|position| {
            self.motor.velocity().map(|velocity| AxisSample {
                position: position.as_radians() * self.output_per_motor,
                velocity: velocity * (core::f64::consts::TAU / 60.0) * self.output_per_motor,
            })
        });
        match sample {
            Ok(sample) => self.apply(sample, target, dt),
            Err(error) => {
                let _ = self.stop();
                Err(MotorError::Port(error))
            }
        }
    }

    pub fn stop(&mut self) -> Result<(), PortError> {
        self.controller.reset();
        self.motor.brake(self.brake)
    }
}
