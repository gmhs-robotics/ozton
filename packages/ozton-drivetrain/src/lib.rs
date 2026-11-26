//! A motion model, the robot's physical geometry, and its tracking system.

mod braking;
pub use braking::{BrakeMode, BrakeMotors, HasBraking, MotorAddress, WheelActuation, WheelCommand};
pub mod device;
mod execution;
pub mod model;
#[cfg(feature = "vex")]
pub use execution::DriveMotionFault;
pub use execution::{MotionExecution, RangeResetDrive};
pub use model::{Drive, LateralDrive};
pub use ozton_math::path::clearance::Footprint;
pub use ozton_tracking::odometric::WheelFeedback;

mod drivetrain;
pub use drivetrain::Drivetrain;
#[cfg(feature = "vex")]
pub use drivetrain::vex::MotorGroups;
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
    fn localization_mut(&mut self) -> impl std::ops::DerefMut<Target = Self::Localization>;
}
