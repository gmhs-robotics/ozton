//! Two sides of motors driven by normalized voltage.
use vexide::{
    prelude::Motor,
    smart::{PortError, motor::BrakeMode},
};

use super::{DrivetrainModel, Tank};
use crate::MotorGroups;

/// Motors on a side receive the same fraction of their rated voltage.
pub struct Differential {
    sides: [Box<[Motor]>; 2],
}

impl Differential {
    pub fn new(
        left: impl IntoIterator<Item = Motor>,
        right: impl IntoIterator<Item = Motor>,
    ) -> Self {
        Self {
            sides: [left.into_iter().collect(), right.into_iter().collect()],
        }
    }
}

impl DrivetrainModel for Differential {
    type Error = PortError;
}

impl MotorGroups for Differential {
    fn motor_groups(&self) -> &[Box<[Motor]>] {
        &self.sides
    }

    fn motor_groups_mut(&mut self) -> &mut [Box<[Motor]>] {
        &mut self.sides
    }
}

impl Tank for Differential {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        if !left.is_finite() || !right.is_finite() || (left == 0.0 && right == 0.0) {
            return self.stop_now();
        }

        self.sides
            .iter_mut()
            .zip(ozton_math::desaturate([left, right], 1.0))
            .fold(Ok(()), |result, (side, power)| {
                result.and(crate::each_motor(side, |motor| {
                    if power == 0.0 {
                        motor.brake(BrakeMode::Coast)
                    } else {
                        motor.set_voltage(power * motor.max_voltage())
                    }
                }))
            })
    }

    fn stop_now(&mut self) -> Result<(), PortError> {
        crate::coast(&mut self.sides)
    }
}
