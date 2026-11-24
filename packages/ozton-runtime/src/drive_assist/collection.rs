//! Assist state stored inline in installation order.

use std::time::Instant;

use ozton_drivetrain::Drive;

use super::{AssistContext, DriveAssist};

/// The owned assist tuple and timing state shared across borrowed drive views.
pub struct DriveAssists<A> {
    pub(super) items: A,
    pub(super) last_tick: Option<(Instant, bool)>,
}

impl<A> DriveAssists<A> {
    pub const fn new(items: A) -> Self {
        Self {
            items,
            last_tick: None,
        }
    }
}

/// Invalidate any assist state when drive output is stopped externally.
pub trait ResetDriveAssists {
    fn reset_drive_assists(&mut self);
}

impl ResetDriveAssists for () {
    fn reset_drive_assists(&mut self) {}
}

impl<A> ResetDriveAssists for DriveAssists<A> {
    fn reset_drive_assists(&mut self) {
        self.last_tick = None;
    }
}

macro_rules! tuple_assists {
    ($($index:tt : $assist:ident),+ $(,)?) => {
        impl<M: Drive, T, L, $($assist: DriveAssist<M, T, L>),+> DriveAssist<M, T, L>
            for ($($assist,)+)
        {
            fn needs_orientation(&self) -> bool {
                false $(|| self.$index.needs_orientation())+
            }

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
