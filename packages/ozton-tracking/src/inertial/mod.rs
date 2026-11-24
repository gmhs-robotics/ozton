//! Inertial tracking sources.

#[cfg(feature = "vex")]
pub mod imu;
#[cfg(feature = "vex")]
pub use imu::InertialSensorTracking;
use ozton_math::uom::si::f64::Angle;

/// Orientation read once for a control tick. Heading follows the IMU's
/// clockwise convention, while roll and pitch follow VEX body axes.
#[derive(Debug, Clone, Copy)]
pub struct Orientation {
    pub port: u8,
    pub heading: Option<Angle>,
    pub pitch: Option<Angle>,
    pub roll: Option<Angle>,
    pub yaw: Option<Angle>,
}
