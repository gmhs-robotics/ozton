//! Optional driver input control and feedback assists for any Ozton drive.
//!
//! A custom drive and assist can be composed using only the interfaces they need.
//!
//! ```
//! use std::convert::Infallible;
//!
//! use ozton_assist::{AssistContext, DriveAssist, DriveAssists};
//! use ozton_drivetrain::Drive;
//! use ozton_math::geometry::DVec2;
//!
//! #[derive(Default)]
//! struct SimDrive(DVec2);
//!
//! impl Drive for SimDrive {
//!     type Error = Infallible;
//!
//!     fn drive_vector(&mut self, translation: DVec2, _turn: f64) -> Result<(), Self::Error> {
//!         self.0 = translation;
//!         Ok(())
//!     }
//! }
//!
//! struct LimitForward;
//!
//! impl<D: Drive> DriveAssist<D> for LimitForward {
//!     fn apply(&mut self, axes: &mut [f64; 2], _: &AssistContext<'_, D>) {
//!         axes[0] *= 0.5;
//!     }
//! }
//!
//! let mut drive = SimDrive::default();
//! DriveAssists::new(LimitForward).apply_to(&mut drive).arcade(1.0, 0.0)?;
//! assert_eq!(drive.0, DVec2::new(0.0, 0.5));
//! # Ok::<(), Infallible>(())
//! ```
use std::time::Duration;

use ozton_drivetrain::Drive;

mod balance;
mod curvature;
mod heading;
mod view;

pub use balance::MotorBalanceAssist;
pub use curvature::CurvatureDrive;
pub use heading::{AntiTipAssist, HeadingHoldAssist, TiltAxis};
pub use view::{AssistedDrive, DriveAssists};

/// Observations and estimates borrowed from the drive for a single command.
pub struct AssistContext<'a, D: ?Sized> {
    pub drive: &'a D,
    pub elapsed: Duration,
    /// Whether the caller supplies physical wheel velocity targets.
    pub wheel_velocity: bool,
}

/// Transform forward/turn axes or adjust the resulting side commands.
pub trait DriveAssist<D: Drive + ?Sized> {
    fn apply(&mut self, _axes: &mut [f64; 2], _context: &AssistContext<'_, D>) {}

    fn wheels(
        &mut self,
        _context: &AssistContext<'_, D>,
        _wheels: &mut [f64; 2],
    ) -> Result<(), D::Error> {
        Ok(())
    }

    fn reset(&mut self) {}
}

impl<D: Drive + ?Sized> DriveAssist<D> for () {}

macro_rules! tuple_assists {
    ($($index:tt : $assist:ident),+ $(,)?) => {
        impl<Host: Drive + ?Sized, $($assist: DriveAssist<Host>),+> DriveAssist<Host>
            for ($($assist,)+)
        {
            fn apply(&mut self, axes: &mut [f64; 2], context: &AssistContext<'_, Host>) {
                $(self.$index.apply(axes, context);)+
            }

            fn wheels(
                &mut self,
                context: &AssistContext<'_, Host>,
                wheels: &mut [f64; 2],
            ) -> Result<(), Host::Error> {
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
