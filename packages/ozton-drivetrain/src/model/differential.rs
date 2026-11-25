//! Two sides of motors driven by normalized voltage.
use glam::DVec2;
use ozton_tracking::odometric::SharedMotors;
use vexide::{
    prelude::Motor,
    smart::{PortError, SmartDevice, motor::MotorType},
};

use super::{Balancing, Drive, MotionGeometry, UnboundedMotion, arcade_mix};
use crate::{BrakeMode, BrakeMotors, Footprint, HasBraking, MotorAddress, MotorGroups};

/// Motors on a side receive the same fraction of their rated voltage.
pub struct Differential<G = ()> {
    sides: [SharedMotors; 2],
    geometry: G,
}

impl Differential {
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
            geometry: (),
        }
    }
}

impl<G> Differential<G> {
    pub fn shared_motors(&self) -> &[SharedMotors; 2] {
        &self.sides
    }

    /// Configure the physical chassis footprint for field-bounded motion.
    pub fn with_footprint(self, footprint: Footprint) -> Differential<Footprint> {
        Differential {
            sides: self.sides,
            geometry: footprint,
        }
    }

    /// Explicitly opt into pose motion without a field clearance envelope.
    pub fn with_unbounded_motion(self) -> Differential<UnboundedMotion> {
        Differential {
            sides: self.sides,
            geometry: UnboundedMotion,
        }
    }
}

impl<G: MotionGeometry> Differential<G> {
    pub fn footprint(&self) -> Option<Footprint> {
        self.geometry.footprint()
    }
}

impl<G> MotorGroups for Differential<G> {
    type WheelSpeeds = [ozton_math::uom::si::f64::Velocity; 2];
    type WheelVoltages = [ozton_math::uom::si::f64::ElectricPotential; 2];

    fn motor_groups(&self) -> &[SharedMotors] {
        &self.sides
    }

    fn motor_groups_mut(&mut self) -> &mut [SharedMotors] {
        &mut self.sides
    }
}

impl<G> Drive for Differential<G> {
    type Error = PortError;

    fn drive_vector(&mut self, vector: DVec2, turn: f64) -> Result<(), PortError> {
        crate::normalized_voltage(&mut self.sides, arcade_mix(vector.y, turn))
    }
}

impl<G> HasBraking for Differential<G> {
    type BrakeError = PortError;

    fn apply_brake(&mut self, mode: BrakeMode) -> Result<(), Self::BrakeError> {
        self.brake_motors(|_| Some(mode))
    }
}

impl<G> BrakeMotors for Differential<G> {
    fn brake_motors(
        &mut self,
        select: impl FnMut(MotorAddress) -> Option<BrakeMode>,
    ) -> Result<(), Self::BrakeError> {
        crate::model::actuation::brake_groups(&mut self.sides, select)
    }
}

impl<G> Balancing for Differential<G> {
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
