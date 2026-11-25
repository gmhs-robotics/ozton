//! Actuation and scheduling capabilities used by motion controllers.

use std::{
    future::Future,
    time::{Duration, Instant},
};

use ozton_localization::LocalizationEstimate;
use ozton_math::{path::clearance::Footprint, uom::si::f64::Velocity};

use crate::{HasLocalization, model::DriveEncoders};

#[cfg(feature = "vex")]
mod integration;
#[cfg(feature = "vex")]
pub use integration::DriveMotionFault;

/// Execute wheel commands using a localized drive and its runtime.
/// Pose, feedback and reset state belong to the localization estimate.
pub trait MotionExecution: DriveEncoders + HasLocalization<Localization: LocalizationEstimate> {
    type Error;

    /// None means the implementation explicitly permits unbounded motion.
    fn footprint(&self) -> Option<Footprint>;
    fn drive_wheels(&mut self, speeds: [Velocity; 2]) -> Result<(), Self::Error>;
    /// Resting stop, which may use passive coast.
    fn brake(&mut self) -> Result<(), Self::Error>;
    /// Immediate active stop. Implementations must not silently coast.
    fn emergency_brake(&mut self) -> Result<(), Self::Error>;

    /// Yield until the monotonic deadline using the platform executor.
    fn wait_until(&mut self, deadline: Instant) -> impl Future<Output = ()>;

    /// False when a competition or application cancellation forbids output.
    fn motion_active(&self) -> bool;

    fn log_step(&mut self, _elapsed: Duration, _dt: Duration) {}
}

/// Marker for motion execution with statically available range observations.
///
/// ```compile_fail
/// use ozton_drivetrain::{MotionExecution, RangeResetDrive};
/// fn require<D: RangeResetDrive>() {}
/// fn without_ranges<D: MotionExecution>() {
///     require::<D>();
/// }
/// ```
pub trait RangeResetDrive: MotionExecution {}
