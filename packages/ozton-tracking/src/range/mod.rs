//! Distance sensors and their observations.

#[cfg(feature = "vex")]
pub mod distance;
#[cfg(feature = "vex")]
pub use distance::{DistanceTracker, Distances};
pub use ozton_math::localization::observation::{MAX_RANGES, RangeObservation, RangeSensorConfig};
