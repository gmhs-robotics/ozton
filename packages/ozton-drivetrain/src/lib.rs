//! A motion model, the robot's physical geometry, and its tracking system.

pub mod model;

pub use model::{Drive, LateralDrive};
pub use ozton_math::path::clearance::Footprint;
pub use ozton_tracking::odometric::WheelFeedback;

#[cfg(feature = "vex")]
mod vex;
#[cfg(feature = "vex")]
mod wheels;
#[cfg(feature = "vex")]
pub use ozton_tracking::odometric::{DriveConfigError, SharedMotors, WheelGearing};
#[cfg(feature = "vex")]
pub(crate) use vex::normalized_voltage;
#[cfg(feature = "vex")]
pub use vex::{Drivetrain, MotorGroups};
#[cfg(feature = "vex")]
pub use wheels::WheelSpeedError;

/// The localization estimate exposed by a drive.
pub trait HasLocalization {
    type Localization;

    fn localization(&self) -> impl std::ops::Deref<Target = Self::Localization>;
}
