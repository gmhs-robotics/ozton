//! Hardware-independent drive assistance contract.
use std::time::Duration;

use ozton_math::{desaturate, localization::observation::Orientation};

use crate::model::{Balancing, Drive};

/// Observations and estimates borrowed from the drive for a single command.
pub struct AssistContext<'a, M, T, L> {
    pub model: &'a M,
    pub tracking: &'a T,
    pub localization: &'a L,
    pub orientation: Option<Orientation>,
    pub elapsed: Duration,
    /// Whether the caller supplies physical wheel velocity targets.
    pub wheel_velocity: bool,
}

/// Transform forward/turn axes or adjust the resulting side commands.
pub trait DriveAssist<M: Drive, T, L> {
    fn needs_orientation(&self) -> bool {
        false
    }

    fn apply(&mut self, _axes: &mut [f64; 2], _context: &AssistContext<'_, M, T, L>) {}

    fn wheels(
        &mut self,
        _context: &AssistContext<'_, M, T, L>,
        _wheels: &mut [f64; 2],
    ) -> Result<(), M::Error> {
        Ok(())
    }

    fn reset(&mut self) {}
}

impl<M: Drive, T, L> DriveAssist<M, T, L> for () {}

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

