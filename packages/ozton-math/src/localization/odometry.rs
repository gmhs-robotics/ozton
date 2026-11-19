//! Tracking-wheel correction relative to The Origin in the right/forward frame.
use glam::DVec2;

use crate::geometry::BodyDelta;

/// Forward wheel: (forward-positive travel, X-right position from The Origin).
/// Lateral wheel: (right-positive travel, Y-forward position from The Origin).
/// Pure CCW rotation produces forward travel X·yaw and right travel -Y·yaw.
pub fn body_delta(forward: (f64, f64), lateral: Option<(f64, f64)>, heading: f64) -> BodyDelta {
    BodyDelta {
        translation: DVec2::new(
            lateral.map_or(0.0, |(travel, offset)| travel + offset * heading),
            forward.0 - forward.1 * heading,
        ),
        heading,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn wheel_positions_use_right_forward_offsets_and_cancel_pure_rotation() {
        for yaw in [-0.4, 0.4] {
            for forward_x in [-0.15, 0.15] {
                for lateral_y in [-0.08, 0.08] {
                    let stationary = body_delta(
                        (forward_x * yaw, forward_x),
                        Some((-lateral_y * yaw, lateral_y)),
                        yaw,
                    );
                    assert!(stationary.translation.length() < 1e-12);
                    let moving = body_delta(
                        (0.3 + forward_x * yaw, forward_x),
                        Some((0.2 - lateral_y * yaw, lateral_y)),
                        yaw,
                    );
                    assert!(moving.translation.distance(DVec2::new(0.2, 0.3)) < 1e-12);
                }
            }
        }
    }
}
