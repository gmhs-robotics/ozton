//! Robot-frame tracking wheel measurements.

use glam::DVec2;

use crate::geometry::BodyDelta;

/// Corrects tracking wheel travel for rotation about the robot origin.
/// A forward wheel offset is positive left, a lateral wheel offset positive forward.
pub fn body_delta(forward: (f64, f64), lateral: Option<(f64, f64)>, heading: f64) -> BodyDelta {
    BodyDelta {
        translation: DVec2::new(
            forward.0 + forward.1 * heading,
            lateral.map_or(0.0, |(travel, offset)| travel - offset * heading),
        ),
        heading,
    }
}
