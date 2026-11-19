//! Explicit right/forward/up installation coordinates and planar conversions.
use ozton_math::{localization::range::Mount, units::Length};
use vexide::math::Angle;

/// A sensor or wheel position relative to CC, preserving its mounting height.
/// This is a location, not an IMU attitude or an inferred odometry measurement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RobotPoint {
    pub x_right_m: f64,
    pub y_forward_m: f64,
    pub z_up_m: f64,
}

impl RobotPoint {
    pub const CC: Self = Self {
        x_right_m: 0.0,
        y_forward_m: 0.0,
        z_up_m: 0.0,
    };

    /// Creates a right/forward/up point relative to CC from physical lengths.
    pub const fn new(right: Length, forward: Length, up: Length) -> Self {
        Self {
            x_right_m: right.as_meters(),
            y_forward_m: forward.as_meters(),
            z_up_m: up.as_meters(),
        }
    }

    /// Validates coordinates before a static mount is used.
    pub const fn is_finite(self) -> bool {
        self.x_right_m.is_finite() && self.y_forward_m.is_finite() && self.z_up_m.is_finite()
    }

    /// Coordinates locate the device from CC, rather than CC from the device.
    pub fn from_meters(x_right_m: f64, y_forward_m: f64, z_up_m: f64) -> Option<Self> {
        [x_right_m, y_forward_m, z_up_m]
            .iter()
            .all(|v| v.is_finite())
            .then_some(Self {
                x_right_m,
                y_forward_m,
                z_up_m,
            })
    }

    /// Adapts a horizontal distance beam to the canonical right/forward planar frame.
    /// Zero bearing faces forward; positive bearing turns left about robot +Z.
    /// Height remains in this point; tilted beams require a separate 3D model.
    pub fn horizontal_range_mount(self, bearing_ccw_from_forward: Angle) -> Option<Mount> {
        if ![
            self.x_right_m,
            self.y_forward_m,
            self.z_up_m,
            bearing_ccw_from_forward.as_radians(),
        ]
        .iter()
        .all(|v| v.is_finite())
        {
            return None;
        }
        Some(Mount::new(
            self.x_right_m,
            self.y_forward_m,
            bearing_ccw_from_forward.as_radians(),
        ))
    }

    /// A forward wheel offset is its X-right coordinate from CC.
    pub const fn forward_wheel_offset_m(self) -> f64 {
        self.x_right_m
    }

    /// A lateral wheel offset is its Y-forward coordinate from CC.
    pub const fn lateral_wheel_offset_m(self) -> f64 {
        self.y_forward_m
    }
    /// The localizer and wheeled tracker share the same Y-forward lateral offset.
    pub const fn localizer_lateral_offset_m(self) -> f64 {
        self.y_forward_m
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mounting_positions_convert_axes_without_losing_height() {
        let point = RobotPoint::from_meters(-0.1, 0.2, 0.3).unwrap();
        let mount = point.horizontal_range_mount(Angle::ZERO).unwrap();
        assert_eq!(mount.offset.x, -0.1);
        assert_eq!(mount.offset.y, 0.2);
        assert_eq!(mount.heading, 0.0);
        assert_eq!(point.z_up_m, 0.3);
        assert_eq!(point.forward_wheel_offset_m(), -0.1);
        assert_eq!(point.lateral_wheel_offset_m(), 0.2);
        assert_eq!(RobotPoint::CC.forward_wheel_offset_m(), 0.0);
    }

    #[test]
    fn beam_bearings_and_invalid_coordinates_keep_explicit_semantics() {
        let point = RobotPoint::CC;
        assert_eq!(
            point
                .horizontal_range_mount(Angle::from_degrees(-90.0))
                .unwrap()
                .heading,
            -core::f64::consts::FRAC_PI_2
        );
        assert!(
            point
                .horizontal_range_mount(Angle::from_radians(f64::NAN))
                .is_none()
        );
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(RobotPoint::from_meters(invalid, 0.0, 0.0).is_none());
            assert!(RobotPoint::from_meters(0.0, invalid, 0.0).is_none());
            assert!(RobotPoint::from_meters(0.0, 0.0, invalid).is_none());
        }
    }
}
