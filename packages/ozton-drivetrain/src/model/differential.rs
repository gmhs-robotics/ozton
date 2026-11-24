//! Two sides of motors driven by normalized voltage.
use glam::DVec2;
use ozton_tracking::odometric::SharedMotors;
use vexide::{
    prelude::Motor,
    smart::{PortError, SmartDevice, motor::MotorType},
};

use super::{Balancing, Drive, arcade_mix};
use crate::{Footprint, MotorGroups};

/// Motors on a side receive the same fraction of their rated voltage.
pub struct Differential {
    sides: [SharedMotors; 2],
    footprint: Option<Footprint>,
}

impl Differential {
    pub const MAX_V5_VOLTAGE: f64 = Motor::V5_MAX_VOLTAGE;

    pub fn new(
        left: impl IntoIterator<Item = Motor>,
        right: impl IntoIterator<Item = Motor>,
    ) -> Self {
        Self::from_shared(
            left.into_iter().collect::<SharedMotors>(),
            right.into_iter().collect::<SharedMotors>(),
        )
    }

    pub fn from_shared(left: impl Into<SharedMotors>, right: impl Into<SharedMotors>) -> Self {
        Self {
            sides: [left.into(), right.into()],
            footprint: None,
        }
    }

    pub fn shared_motors(&self) -> &[SharedMotors; 2] {
        &self.sides
    }

    pub fn with_footprint(mut self, footprint: Footprint) -> Self {
        self.footprint = Some(footprint);
        self
    }

    pub fn footprint(&self) -> Option<Footprint> {
        self.footprint
    }
}

impl MotorGroups for Differential {
    type WheelSpeeds = [ozton_math::uom::si::f64::Velocity; 2];
    type WheelVoltages = [ozton_math::uom::si::f64::ElectricPotential; 2];

    fn motor_groups(&self) -> &[SharedMotors] {
        &self.sides
    }

    fn motor_groups_mut(&mut self) -> &mut [SharedMotors] {
        &mut self.sides
    }
}

impl Drive for Differential {
    type Error = PortError;

    fn drive_vector(&mut self, vector: DVec2, turn: f64) -> Result<(), PortError> {
        crate::normalized_voltage(&mut self.sides, arcade_mix(vector.y, turn))
    }
}

impl Balancing for Differential {
    fn balance(&self, command: &mut [f64; 2]) -> Result<(), PortError> {
        if *command == [0.0; 2] {
            return Ok(());
        }
        let mut fault = None;
        let availability = self.sides.each_ref().map(|side| {
            let mut available = 0;
            let mut total = 0;
            for motor in side.borrow().iter() {
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
