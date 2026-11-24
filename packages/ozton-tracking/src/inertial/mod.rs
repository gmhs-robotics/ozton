//! Inertial tracking sources.

#[cfg(feature = "vex")]
pub mod imu;
#[cfg(feature = "vex")]
pub use imu::InertialSensorTracking;
pub use ozton_math::localization::observation::Orientation;
