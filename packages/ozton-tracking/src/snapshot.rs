//! Timestamped cumulative device observations.

use std::time::Duration;

use ozton_math::uom::si::f64::{Angle, Length};

/// One tick's cumulative encoder positions in meters and heading in radians.
/// Unavailable or uncalibrated devices supply None.
#[derive(Debug, Clone, Copy, Default)]
pub struct Snapshot {
    pub time: Duration,
    pub drive: [Option<Length>; 2],
    pub forward: Option<Length>,
    pub lateral: Option<Length>,
    pub imu: Option<Angle>,
}
