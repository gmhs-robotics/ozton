//! Motor capacity balancing assist.

use ozton_math::desaturate;

use super::{AssistContext, DriveAssist};
use crate::model::Balancing;

/// Use the drivetrain model's motor-loss policy before writing outputs.
#[derive(Clone, Copy, Default)]
pub struct MotorBalanceAssist;

impl<M: Balancing, T, L> DriveAssist<M, T, L> for MotorBalanceAssist {
    fn wheels(
        &mut self,
        context: &AssistContext<'_, M, T, L>,
        wheels: &mut [f64; 2],
    ) -> Result<(), M::Error> {
        *wheels = desaturate(*wheels, 1.0);
        context.model.balance(wheels)
    }
}
