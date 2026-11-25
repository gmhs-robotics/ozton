//! Distance sensors and their observations.

#[cfg(feature = "vex")]
pub mod distance;
#[cfg(feature = "vex")]
pub use distance::{DistanceTracker, Distances};
pub use ozton_math::localization::observation::{MAX_RANGES, RangeObservation, RangeRoles, RangeSensorConfig};

/// A concrete source guaranteed to provide a configured range mount.
/// Implement this for custom ranging sources as well as hardware adapters.
pub trait RangeSource: crate::TrackingSource {
    fn range_config(&self) -> RangeSensorConfig;
}
