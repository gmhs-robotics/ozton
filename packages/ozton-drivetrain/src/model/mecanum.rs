use glam::DVec2 as Vec2;
use ozton_math::desaturate;
use vexide::smart::{PortError, motor::Motor};

use super::{DrivetrainModel, Holonomic, Tank};

/// A mecanum drivetrain owning its four motor groups.
pub struct Mecanum {
    motors: [Box<[Motor]>; 4],
}

impl Mecanum {
    pub fn new(
        front_left: impl Into<Box<[Motor]>>,
        front_right: impl Into<Box<[Motor]>>,
        back_left: impl Into<Box<[Motor]>>,
        back_right: impl Into<Box<[Motor]>>,
    ) -> Self {
        Self {
            motors: [
                front_left.into(),
                front_right.into(),
                back_left.into(),
                back_right.into(),
            ],
        }
    }

    fn drive_wheels(&mut self, powers: [f64; 4]) -> Result<(), PortError> {
        let mut result = Ok(());
        for (motors, power) in self.motors.iter_mut().zip(powers) {
            for motor in motors.iter_mut() {
                if let Err(error) = motor.set_voltage(power * motor.max_voltage()) {
                    result = Err(error);
                }
            }
        }
        result
    }
}

impl DrivetrainModel for Mecanum {
    type Error = PortError;
}

impl Holonomic for Mecanum {
    fn drive_vector(&mut self, vector: Vec2, turn: f64) -> Result<(), Self::Error> {
        self.drive_wheels(desaturate(
            [
                vector.y + vector.x + turn,
                vector.y - vector.x - turn,
                vector.y - vector.x + turn,
                vector.y + vector.x - turn,
            ],
            1.0,
        ))
    }
}

impl Tank for Mecanum {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), Self::Error> {
        self.drive_wheels([left, right, left, right])
    }
}
