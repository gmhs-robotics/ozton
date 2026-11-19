//! Tracking-wheel correction relative to The Origin in the right/forward frame.
use glam::DVec2;
use uom::si::f64::{Angle, Length};

use crate::geometry::BodyDelta;

/// Forward wheel: (forward-positive travel, X-right position from The Origin).
/// Lateral wheel: (right-positive travel, Y-forward position from The Origin).
/// Pure CCW rotation produces forward travel X·yaw and right travel -Y·yaw.
pub fn body_delta(
    forward: (Length, Length),
    lateral: Option<(Length, Length)>,
    heading: Angle,
) -> BodyDelta {
    BodyDelta {
        translation: DVec2::new(
            lateral.map_or(0.0, |(travel, offset)| {
                travel.value + offset.value * heading.value
            }),
            forward.0.value - forward.1.value * heading.value,
        ),
        heading: heading.value,
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
                        (crate::si(forward_x * yaw), crate::si(forward_x)),
                        Some((crate::si(-lateral_y * yaw), crate::si(lateral_y))),
                        crate::si(yaw),
                    );
                    assert!(stationary.translation.length() < 1e-12);
                    let moving = body_delta(
                        (crate::si(0.3 + forward_x * yaw), crate::si(forward_x)),
                        Some((crate::si(0.2 - lateral_y * yaw), crate::si(lateral_y))),
                        crate::si(yaw),
                    );
                    assert!(moving.translation.distance(DVec2::new(0.2, 0.3)) < 1e-12);
                }
            }
        }
    }
}
