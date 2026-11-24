use glam::DVec2;
use ozton_tracking::odometric::SharedMotors;
use vexide::{prelude::Motor, smart::PortError};

use super::{Drive, LateralDrive};
use crate::MotorGroups;

/// Motor groups in front-left, front-right, back-left, back-right order.
pub struct Mecanum {
    motors: [SharedMotors; 4],
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
}

impl MotorGroups for Mecanum {
    type WheelSpeeds = [ozton_math::uom::si::f64::Velocity; 4];

    fn motor_groups(&self) -> &[SharedMotors] {
        &self.motors
    }

    fn motor_groups_mut(&mut self) -> &mut [SharedMotors] {
        &mut self.motors
    }
}

impl Drive for Mecanum {
    type Error = PortError;

    fn drive_vector(&mut self, vector: DVec2, turn: f64) -> Result<(), PortError> {
        crate::normalized_voltage(&mut self.motors, mix(vector.x, vector.y, turn))
    }
}

impl LateralDrive for Mecanum {}

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
