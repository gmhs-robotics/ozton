//! Two sides of motors driven by normalized voltage.
use vexide::{
    prelude::Motor,
    smart::{PortError, SmartDevice, motor::BrakeMode},
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

    fn drive_driver_tank(
        &mut self,
        left: f64,
        right: f64,
        balance_disconnected: bool,
    ) -> Result<(), PortError> {
        if !balance_disconnected || (left == 0.0 && right == 0.0) {
            return self.drive_tank(left, right);
        }
        let mut fault = None;
        let availability = self.sides.each_ref().map(|side| {
            let connected = side
                .iter()
                .filter(|motor| match motor.validate_port() {
                    Ok(()) => true,
                    Err(error) => {
                        fault.get_or_insert(error);
                        false
                    }
                })
                .count();
            (connected, side.len())
        });
        let [left, right] = balance_motor_output([left, right], availability);
        self.drive_tank(left, right).and(fault.map_or(Ok(()), Err))
    }

    fn stop_now(&mut self) -> Result<(), PortError> {
        crate::coast(&mut self.sides)
    }
}

/// Reduce the healthier side in proportion to the available fraction of each side's motors.
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
    }
}
