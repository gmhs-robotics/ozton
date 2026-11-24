//! Vexide actuator adapter for a reusable Ozton control axis.

use std::time::Duration;

use ozton_math::{
    control::axis::{AxisController, AxisSample, Target},
    uom::si::{electric_potential::volt, f64::ElectricPotential},
};
use vexide::{
    prelude::Motor,
    smart::{PortError, motor::BrakeMode},
};

use crate::device::{MotorDevice, VelocityMotor};

#[derive(Debug)]
pub enum MotorError {
    Port(PortError),
    InvalidFeedback,
}

/// Controls a motor in output radians and radians per second.
pub struct PidMotor<M: MotorDevice = Motor> {
    pub motor: M,
    controller: AxisController,
    output_per_motor: f64,
    brake: BrakeMode,
}

impl<M: MotorDevice> PidMotor<M> {
    pub fn new(
        motor: M,
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

    /// Configures a mechanism with a 1:1 output ratio and active holding on stop.
    pub fn builder(motor: M, controller: AxisController) -> PidMotorBuilder<M> {
        PidMotorBuilder::new(motor, controller)
    }

    /// Use this with either the integrated encoder or an independently sampled joint encoder.
    pub fn apply(
        &mut self,
        sample: AxisSample,
        target: Target,
        dt: Duration,
    ) -> Result<ElectricPotential, MotorError> {
        let Some(volts) = self.controller.update(sample, target, dt) else {
            self.stop().map_err(MotorError::Port)?;
            return Err(MotorError::InvalidFeedback);
        };
        let maximum = self.motor.max_voltage();
        if !maximum.value.is_finite() || maximum.value <= 0.0 {
            self.stop().map_err(MotorError::Port)?;
            return Err(MotorError::InvalidFeedback);
        }
        let voltage =
            ElectricPotential::new::<volt>(volts.value.clamp(-maximum.value, maximum.value));
        if let Err(error) = self.motor.set_voltage(voltage) {
            let _ = self.stop();
            return Err(MotorError::Port(error));
        }
        Ok(voltage)
    }

    pub fn update(&mut self, target: Target, dt: Duration) -> Result<ElectricPotential, MotorError>
    where
        M: VelocityMotor,
    {
        let sample = self.motor.position().and_then(|position| {
            self.motor.velocity().map(|velocity| AxisSample {
                position: position * self.output_per_motor,
                velocity: velocity * self.output_per_motor,
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

/// Configures device gearing and stop behavior before creating an actuator.
/// Use Coast for mechanisms that must not actively press against a stowed stop.
pub struct PidMotorBuilder<M: MotorDevice> {
    motor: M,
    controller: AxisController,
    output_per_motor: f64,
    brake: BrakeMode,
}

impl<M: MotorDevice> PidMotorBuilder<M> {
    pub fn new(motor: M, controller: AxisController) -> Self {
        Self {
            motor,
            controller,
            output_per_motor: 1.0,
            brake: BrakeMode::Hold,
        }
    }

    /// Positive output revolutions per motor revolution.
    #[must_use]
    pub fn with_output_ratio(mut self, output_per_motor: f64) -> Self {
        self.output_per_motor = output_per_motor;
        self
    }

    #[must_use]
    pub fn with_brake(mut self, brake: BrakeMode) -> Self {
        self.brake = brake;
        self
    }

    pub fn build(self) -> Option<PidMotor<M>> {
        PidMotor::new(
            self.motor,
            self.controller,
            self.output_per_motor,
            self.brake,
        )
    }
}

#[cfg(test)]
mod tests {
    use ozton_math::{
        control::{Pid, axis::AxisConfig},
        uom::si::{
            angular_acceleration::radian_per_second_squared,
            angular_velocity::radian_per_second,
            f64::{Angle, AngularAcceleration, AngularVelocity},
        },
    };

    use super::*;

    #[derive(Default)]
    struct FakeMotor {
        position: f64,
        rpm: f64,
        voltage: f64,
        maximum: f64,
        brake: Option<BrakeMode>,
    }

    impl MotorDevice for FakeMotor {
        fn position(&self) -> Result<Angle, PortError> {
            Ok(ozton_math::qty!((self.position) rad))
        }
        fn set_position(&mut self, position: Angle) -> Result<(), PortError> {
            self.position = position.value;
            Ok(())
        }
        fn brake(&mut self, mode: BrakeMode) -> Result<(), PortError> {
            self.voltage = 0.0;
            self.brake = Some(mode);
            Ok(())
        }
        fn max_voltage(&self) -> ElectricPotential {
            ElectricPotential::new::<volt>(self.maximum)
        }
        fn set_voltage(&mut self, voltage: ElectricPotential) -> Result<(), PortError> {
            self.voltage = voltage.value;
            Ok(())
        }
    }

    impl VelocityMotor for FakeMotor {
        fn velocity(&self) -> Result<AngularVelocity, PortError> {
            Ok(AngularVelocity::new::<radian_per_second>(
                self.rpm * (core::f64::consts::TAU / 60.0),
            ))
        }
    }

    fn controller() -> AxisController {
        AxisConfig::new()
            .with_pid(Pid::new(1.0, 0.0, 0.0, None))
            .build()
            .unwrap()
    }

    #[test]
    fn builder_applies_gearing_rated_voltage_and_passive_stop() {
        let motor = FakeMotor {
            position: 4.0,
            rpm: 60.0,
            maximum: 8.0,
            ..Default::default()
        };
        let mut axis = PidMotor::builder(motor, controller())
            .with_output_ratio(0.25)
            .with_brake(BrakeMode::Coast)
            .build()
            .unwrap();
        let dt = Duration::from_millis(10);
        let position = Target::Position {
            position: ozton_math::qty!(2.0 rad),
            velocity: AngularVelocity::new::<radian_per_second>(0.0),
            acceleration: AngularAcceleration::new::<radian_per_second_squared>(0.0),
        };
        assert_eq!(
            axis.update(position, dt).unwrap(),
            ElectricPotential::new::<volt>(1.0)
        );
        let velocity = Target::Velocity {
            velocity: AngularVelocity::new::<radian_per_second>(core::f64::consts::TAU * 0.25),
            acceleration: AngularAcceleration::new::<radian_per_second_squared>(0.0),
        };
        assert!(axis.update(velocity, dt).unwrap().value.abs() < 1e-12);
        assert_eq!(
            axis.apply(
                AxisSample {
                    position: ozton_math::qty!(0.0 rad),
                    velocity: AngularVelocity::new::<radian_per_second>(0.0)
                },
                Target::Velocity {
                    velocity: AngularVelocity::new::<radian_per_second>(100.0),
                    acceleration: AngularAcceleration::new::<radian_per_second_squared>(0.0)
                },
                dt
            )
            .unwrap(),
            ElectricPotential::new::<volt>(8.0)
        );
        axis.stop().unwrap();
        assert_eq!(axis.motor.voltage, 0.0);
        assert_eq!(axis.motor.brake, Some(BrakeMode::Coast));
    }

    #[test]
    fn invalid_gearing_and_feedback_fail_closed() {
        for ratio in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(
                PidMotorBuilder::new(FakeMotor::default(), controller())
                    .with_output_ratio(ratio)
                    .build()
                    .is_none()
            );
        }
        let mut axis = PidMotorBuilder::new(FakeMotor::default(), controller())
            .build()
            .unwrap();
        let target = Target::Velocity {
            velocity: AngularVelocity::new::<radian_per_second>(1.0),
            acceleration: AngularAcceleration::new::<radian_per_second_squared>(0.0),
        };
        assert!(matches!(
            axis.update(target, Duration::from_millis(10)),
            Err(MotorError::InvalidFeedback)
        ));
        assert_eq!(axis.motor.voltage, 0.0);
        assert_eq!(axis.motor.brake, Some(BrakeMode::Hold));
    }
}
