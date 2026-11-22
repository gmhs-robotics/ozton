use glam::DVec2;
use vexide::{
    prelude::Motor,
    smart::{PortError, motor::BrakeMode},
};

use super::{DrivetrainModel, Holonomic, Tank};
use crate::MotorGroups;

/// Motor groups in front-left, front-right, back-left, back-right order.
pub struct Mecanum {
    motors: [Box<[Motor]>; 4],
}

impl Mecanum {
    pub fn new(
        front_left: impl IntoIterator<Item = Motor>,
        front_right: impl IntoIterator<Item = Motor>,
        back_left: impl IntoIterator<Item = Motor>,
        back_right: impl IntoIterator<Item = Motor>,
    ) -> Self {
        Self {
            motors: [
                front_left.into_iter().collect(),
                front_right.into_iter().collect(),
                back_left.into_iter().collect(),
                back_right.into_iter().collect(),
            ],
        }
    }

    fn drive_wheels(&mut self, powers: [f64; 4]) -> Result<(), PortError> {
        if powers.iter().any(|power| !power.is_finite()) || powers == [0.0; 4] {
            return self.stop_now();
        }
        self.motors
            .iter_mut()
            .zip(ozton_math::desaturate(powers, 1.0))
            .fold(Ok(()), |result, (motors, power)| {
                result.and(crate::each_motor(motors, |motor| {
                    if power == 0.0 {
                        motor.brake(BrakeMode::Coast)
                    } else {
                        motor.set_voltage(power * motor.max_voltage())
                    }
                }))
            })
    }
}

impl DrivetrainModel for Mecanum {
    type Error = PortError;
}

impl MotorGroups for Mecanum {
    fn motor_groups(&self) -> &[Box<[Motor]>] {
        &self.motors
    }

    fn motor_groups_mut(&mut self) -> &mut [Box<[Motor]>] {
        &mut self.motors
    }
}

impl Holonomic for Mecanum {
    fn drive_vector(&mut self, vector: DVec2, turn: f64) -> Result<(), PortError> {
        self.drive_wheels(mix(vector.x, vector.y, turn))
    }
}

impl Tank for Mecanum {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        self.drive_wheels([left, right, left, right])
    }

    fn stop_now(&mut self) -> Result<(), PortError> {
        crate::coast(&mut self.motors)
    }
}

fn mix(strafe: f64, forward: f64, turn: f64) -> [f64; 4] {
    [
        forward + strafe + turn,
        forward - strafe - turn,
        forward - strafe + turn,
        forward + strafe - turn,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wheel_order_supports_strafe_and_turn() {
        assert_eq!(mix(1.0, 0.0, 0.0), [1.0, -1.0, -1.0, 1.0]);
        assert_eq!(mix(0.0, 0.0, 1.0), [1.0, -1.0, 1.0, -1.0]);
    }
}
