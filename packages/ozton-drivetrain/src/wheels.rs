//! Calibrated wheel velocity actuation.
use core::{error::Error, fmt};

use ozton_tracking::odometric::{SharedMotors, Wheels};
use vexide::smart::PortError;

use crate::{HasBraking, WheelCommand};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WheelSpeedError {
    InvalidCommand,
    Port(PortError),
}

impl fmt::Display for WheelSpeedError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidCommand => f.write_str("Invalid wheel speed command"),
            Self::Port(error) => error.fmt(f),
        }
    }
}

impl Error for WheelSpeedError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Port(error) => Some(error),
            _ => None,
        }
    }
}

pub(crate) fn drive_commands(
    wheels: &Wheels,
    motors: &mut [SharedMotors],
    commands: &[WheelCommand],
) -> Result<(), WheelSpeedError> {
    if commands.len() != motors.len()
        || !wheels.matches(motors)
        || !commands.iter().all(|command| command.is_valid())
    {
        return Err(WheelSpeedError::InvalidCommand);
    }
    let mut result = Ok(());
    for (group, (motors, command)) in motors.iter_mut().zip(commands).enumerate() {
        for (index, motor) in motors.borrow_mut().iter_mut().enumerate() {
            let written = match *command {
                WheelCommand::Brake(mode) => motor.apply_brake(mode),
                WheelCommand::Velocity(speed) => wheels
                    .meters_per_turn(group, index, motor)
                    .and_then(|travel| {
                        motor.set_velocity((speed.value * 60.0 / travel).round() as i32)
                    }),
            };
            result = result.and(written);
        }
    }
    result.map_err(WheelSpeedError::Port)
}
