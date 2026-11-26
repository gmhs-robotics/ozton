//! Motor capacity balancing assist.

use ozton_drivetrain::model::Balancing;
use ozton_math::desaturate;

use super::{AssistContext, DriveAssist};

/// Use the drivetrain model's motor-loss policy before writing outputs.
#[derive(Clone, Copy, Default)]
pub struct MotorBalanceAssist;

impl<D: Balancing + ?Sized> DriveAssist<D> for MotorBalanceAssist {
    fn wheels(
        &mut self,
        context: &AssistContext<'_, D>,
        wheels: &mut [f64; 2],
    ) -> Result<(), D::Error> {
        *wheels = desaturate(*wheels, 1.0);
        context.drive.balance(wheels)
    }
}
