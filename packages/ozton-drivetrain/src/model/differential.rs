//! Two sides of motors driven by normalized voltage.
use vexide::{
    prelude::Motor,
    smart::{PortError, SmartDevice, motor::MotorType},
};

use super::{Balancing, DrivetrainModel, Tank};
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
    type WheelSpeeds = [ozton_math::uom::si::f64::Velocity; 2];

    fn motor_groups(&self) -> &[Box<[Motor]>] {
        &self.sides
    }

    fn motor_groups_mut(&mut self) -> &mut [Box<[Motor]>] {
        &mut self.sides
    }
}

impl Tank for Differential {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        crate::normalized_voltage(&mut self.sides, [left, right])
    }
}

impl Balancing for Differential {
    type Command = [f64; 2];

    fn balance(&self, command: &mut Self::Command) -> Result<(), PortError> {
        if *command == [0.0; 2] {
            return Ok(());
        }
        let mut fault = None;
        let availability = self.sides.each_ref().map(|side| {
            let mut available = 0;
            let mut total = 0;
            for motor in side {
                // Units of 5.5 W, matching the rated EXP and V5 motor outputs.
                let capacity = match motor.motor_type() {
                    MotorType::Exp => 1,
                    MotorType::V5 => 2,
                };
                total += capacity;
                match motor.validate_port() {
                    Ok(()) => available += capacity,
                    Err(error) => {
                        fault.get_or_insert(error);
                    }
                }
            }
            (available, total)
        });
        *command = balance_motor_output(*command, availability);
        fault.map_or(Ok(()), Err)
    }

    fn drive_balanced(&mut self, mut command: Self::Command) -> Result<(), PortError> {
        let balance = self.balance(&mut command);
        self.drive_tank(command[0], command[1]).and(balance)
    }
}

/// Reduce the healthier side in proportion to the available fraction of each side's rated power.
fn balance_motor_output(outputs: [f64; 2], availability: [(usize, usize); 2]) -> [f64; 2] {
    let [left, right] = availability.map(|(connected, total)| {
        if total == 0 {
            0.0
        } else {
            connected as f64 / total as f64
        }
    });
    if left == 0.0 || right == 0.0 {
        return [0.0; 2];
    }
    let common = left.min(right);
    [outputs[0] * (common / left), outputs[1] * (common / right)]
}

#[cfg(test)]
mod tests {
    use super::balance_motor_output;

    #[test]
    fn missing_motors_limit_output_and_a_missing_side_stops_both() {
        let requested = [0.6, -0.3];
        assert_eq!(balance_motor_output(requested, [(3, 3), (2, 2)]), requested);
        assert_eq!(
            balance_motor_output(requested, [(1, 3), (2, 3)]),
            [0.6, -0.15]
        );
        assert_eq!(balance_motor_output(requested, [(3, 3), (0, 3)]), [0.0; 2]);
        assert_eq!(balance_motor_output([1.0; 2], [(3, 5), (5, 5)]), [1.0, 0.6]);
    }
}
