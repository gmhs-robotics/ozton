//! Hardware-independent drive assistance contract.
use std::time::Duration;

use crate::model::Drive;

mod balance;
mod heading;
#[cfg(feature = "vex")]
mod view;

pub use balance::MotorBalanceAssist;
pub use heading::{AntiTipAssist, HeadingHoldAssist, TiltAxis};
#[cfg(feature = "vex")]
pub use view::{AssistedDrivetrain, DriveAssists};

/// Observations and estimates borrowed from the drive for a single command.
pub struct AssistContext<'a, M, T, L> {
    pub model: &'a M,
    pub tracking: &'a T,
    pub localization: &'a L,
    pub elapsed: Duration,
    /// Whether the caller supplies physical wheel velocity targets.
    pub wheel_velocity: bool,
}

/// Transform forward/turn axes or adjust the resulting side commands.
pub trait DriveAssist<M: Drive, T, L> {
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

macro_rules! tuple_assists {
    ($($index:tt : $assist:ident),+ $(,)?) => {
        impl<M: Drive, T, L, $($assist: DriveAssist<M, T, L>),+> DriveAssist<M, T, L>
            for ($($assist,)+)
        {
            fn apply(&mut self, axes: &mut [f64; 2], context: &AssistContext<'_, M, T, L>) {
                $(self.$index.apply(axes, context);)+
            }

            fn wheels(
                &mut self,
                context: &AssistContext<'_, M, T, L>,
                wheels: &mut [f64; 2],
            ) -> Result<(), M::Error> {
                Ok(())$(.and(self.$index.wheels(context, wheels)))+
            }

            fn reset(&mut self) {
                $(self.$index.reset();)+
            }
        }
    };
}

tuple_assists!(0: A);
tuple_assists!(0: A, 1: B);
tuple_assists!(0: A, 1: B, 2: C);
tuple_assists!(0: A, 1: B, 2: C, 3: D);
tuple_assists!(0: A, 1: B, 2: C, 3: D, 4: E);
tuple_assists!(0: A, 1: B, 2: C, 3: D, 4: E, 5: F);
