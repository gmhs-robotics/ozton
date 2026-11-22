//! Calibrated wheel velocity actuation.

use ozton_math::uom::si::f64::Velocity;
use ozton_tracking::encoders::Wheels;
use vexide::{
    prelude::Motor,
    smart::{PortError, motor::BrakeMode},
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WheelSpeedError {
    InvalidCommand,
    MissingWheelGeometry,
    Port(PortError),
}

pub(crate) fn drive_speed(
    wheels: &Wheels,
    motors: &mut [Box<[Motor]>],
    speeds: &[Velocity],
) -> Result<(), WheelSpeedError> {
    if speeds.len() != motors.len()
        || !wheels.matches(motors)
        || speeds.iter().any(|speed| !speed.value.is_finite())
    {
        return Err(WheelSpeedError::InvalidCommand);
    }
    let mut result = Ok(());
    for (group, (motors, speed)) in motors.iter_mut().zip(speeds).enumerate() {
        for (index, motor) in motors.iter_mut().enumerate() {
            let written = if speed.value == 0.0 {
                motor.brake(BrakeMode::Coast)
            } else {
                wheels
                    .meters_per_turn(group, index, motor)
                    .and_then(|travel| {
                        motor.set_velocity((speed.value * 60.0 / travel).round() as i32)
                    })
            };
            result = result.and(written);
        }
    }
    result.map_err(WheelSpeedError::Port)
}
