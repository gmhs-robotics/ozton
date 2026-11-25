//! Motor-group actuation shared by the drivetrain models.
use core::{error::Error, fmt};

use ozton_tracking::odometric::{SharedMotors, Wheels};
use vexide::{
    prelude::Motor,
    smart::{PortError, motor::BrakeMode},
};

use crate::{HasBraking, MotorAddress, WheelCommand};

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

/// Apply selective stops without duplicating the iteration in each model.
pub(crate) fn brake_groups(
    groups: &mut [SharedMotors],
    mut select: impl FnMut(MotorAddress) -> Option<crate::BrakeMode>,
) -> Result<(), PortError> {
    let mut result = Ok(());
    for (group, motors) in groups.iter_mut().enumerate() {
        for (motor, device) in motors.borrow_mut().iter_mut().enumerate() {
            if let Some(mode) = select(MotorAddress { group, motor }) {
                result = result.and(device.apply_brake(mode));
            }
        }
    }
    result
}

pub(crate) fn normalized_voltage<const N: usize>(
    groups: &mut [SharedMotors; N],
    powers: [f64; N],
) -> Result<(), PortError> {
    if powers.iter().any(|power| !power.is_finite()) {
        return coast(groups);
    }
    command_groups(
        groups.iter_mut().zip(ozton_math::desaturate(powers, 1.0)),
        |motor, power| {
            if power == 0.0 {
                motor.brake(BrakeMode::Coast)
            } else {
                motor.set_voltage(power * motor.max_voltage())
            }
        },
    )
}

pub(crate) fn command_groups<'a, T: Copy>(
    groups: impl IntoIterator<Item = (&'a mut SharedMotors, T)>,
    mut command: impl FnMut(&mut Motor, T) -> Result<(), PortError>,
) -> Result<(), PortError> {
    let mut result = Ok(());
    for (group, input) in groups {
        result = result.and(each_motor(&mut group.borrow_mut(), |motor| {
            command(motor, input)
        }));
    }
    result
}

pub(crate) fn each_motor(
    motors: &mut [Motor],
    mut command: impl FnMut(&mut Motor) -> Result<(), PortError>,
) -> Result<(), PortError> {
    // Every motor must receive the command, including after an earlier failure.
    let mut result = Ok(());
    for motor in motors {
        result = result.and(command(motor));
    }
    result
}

pub(crate) fn coast(groups: &mut [SharedMotors]) -> Result<(), PortError> {
    command_groups(groups.iter_mut().zip(std::iter::repeat(())), |motor, ()| {
        motor.brake(BrakeMode::Coast)
    })
}
