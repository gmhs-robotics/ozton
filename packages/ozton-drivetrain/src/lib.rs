//! A motion model, the robot's physical geometry, and its tracking system.

pub mod assist;
mod braking;
pub use braking::{BrakeMode, BrakeMotors, HasBraking, MotorAddress, WheelActuation, WheelCommand};
pub mod device;
pub mod model;
mod pose;
pub use model::{Drive, LateralDrive};
pub use ozton_math::path::clearance::Footprint;
pub use ozton_tracking::odometric::WheelFeedback;
#[cfg(feature = "vex")]
pub use pose::DriveMotionFault;
pub use pose::{PoseDrive, RangeResetDrive};

#[cfg(feature = "vex")]
mod drivetrain;
#[cfg(feature = "vex")]
pub use drivetrain::{Drivetrain, MotorGroups};
#[cfg(feature = "vex")]
pub use model::actuation::WheelSpeedError;
#[cfg(feature = "vex")]
pub(crate) use model::actuation::normalized_voltage;
#[cfg(feature = "vex")]
pub use ozton_tracking::odometric::{DriveConfigError, SharedMotors, WheelGearing};

/// The localization estimate exposed by a drive.
pub trait HasLocalization {
    type Localization;

    fn localization(&self) -> impl std::ops::Deref<Target = Self::Localization>;
}
