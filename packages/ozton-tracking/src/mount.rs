//! Installation coordinates measured from the chassis center of rotation.
use ozton_math::{localization::range::Mount, uom::si::f64::Length};
use vexide::math::Angle;

/// A chassis direction used to label a horizontal measurement axis.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MountDirection {
    Right,
    Forward,
    Left,
    Backward,
}

impl MountDirection {
    const fn right_forward(self) -> (f64, f64) {
        match self {
            Self::Right => (1.0, 0.0),
            Self::Forward => (0.0, 1.0),
            Self::Left => (-1.0, 0.0),
            Self::Backward => (0.0, -1.0),
        }
    }
}

/// The positive X and Y directions of an installation survey.
///
/// Both axes start at The Origin. Z always points up. Use this to enter mount
/// measurements in the survey's axes and convert them to robot coordinates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MountAxes {
    x: MountDirection,
    y: MountDirection,
}

impl MountAxes {
    /// Survey convention with X right and Y forward.
    pub const RIGHT_FORWARD: Self = Self::new(MountDirection::Right, MountDirection::Forward);

    /// Sets the positive measurement directions.
    ///
    /// # Panics
    ///
    /// Panics unless X and Y are perpendicular.
    pub const fn new(x: MountDirection, y: MountDirection) -> Self {
        let (x_right, x_forward) = x.right_forward();
        let (y_right, y_forward) = y.right_forward();
        assert!(x_right * y_right + x_forward * y_forward == 0.0);
        Self { x, y }
    }

    /// Converts survey lengths from The Origin to right/forward/up robot coordinates.
    pub const fn point(self, x: Length, y: Length, up: Length) -> RobotPoint {
        let (x_right, x_forward) = self.x.right_forward();
        let (y_right, y_forward) = self.y.right_forward();
        RobotPoint {
            x_right_m: x.value * x_right + y.value * y_right,
            y_forward_m: x.value * x_forward + y.value * y_forward,
            z_up_m: up.value,
        }
    }
}

/// A sensor or wheel position relative to The Origin, preserving its mounting height.
/// This is a location, not an IMU attitude or an inferred odometry measurement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RobotPoint {
    pub x_right_m: f64,
    pub y_forward_m: f64,
    pub z_up_m: f64,
}

impl RobotPoint {
    /// The Origin, at the chassis center of rotation in the robot frame.
    pub const ORIGIN: Self = Self {
        x_right_m: 0.0,
        y_forward_m: 0.0,
        z_up_m: 0.0,
    };

    /// Creates a right/forward/up point relative to The Origin from physical lengths.
    pub const fn new(right: Length, forward: Length, up: Length) -> Self {
        Self {
            x_right_m: right.value,
            y_forward_m: forward.value,
            z_up_m: up.value,
        }
    }

    /// Validates coordinates before a static mount is used.
    pub const fn is_finite(self) -> bool {
        self.x_right_m.is_finite() && self.y_forward_m.is_finite() && self.z_up_m.is_finite()
    }

    /// Coordinates give the device position relative to The Origin.
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

    /// A forward wheel offset is its X-right coordinate from The Origin.
    pub const fn forward_wheel_offset_m(self) -> f64 {
        self.x_right_m
    }

    /// A lateral wheel offset is its Y-forward coordinate from The Origin.
    pub const fn lateral_wheel_offset_m(self) -> f64 {
        self.y_forward_m
    }
}

#[cfg(test)]
mod tests {
    use ozton_math::si;

    use super::*;

    #[test]
    fn alternate_axes_use_the_same_origin_and_wheel_lever_arms() {
        for (axes, x, y) in [
            (MountAxes::RIGHT_FORWARD, -0.1, 0.2),
            (
                MountAxes::new(MountDirection::Forward, MountDirection::Left),
                0.2,
                0.1,
            ),
            (
                MountAxes::new(MountDirection::Left, MountDirection::Forward),
                0.1,
                0.2,
            ),
            (
                MountAxes::new(MountDirection::Left, MountDirection::Backward),
                0.1,
                -0.2,
            ),
        ] {
            assert_eq!(axes.point(si(0.0), si(0.0), si(0.0)), RobotPoint::ORIGIN);
            let point = axes.point(si(x), si(y), si(0.3));
            assert_eq!(point, RobotPoint::new(si(-0.1), si(0.2), si(0.3)));

            let yaw = 0.4;
            let delta = ozton_math::localization::odometry::body_delta(
                (-0.1 * yaw, point.forward_wheel_offset_m()),
                Some((-0.2 * yaw, point.lateral_wheel_offset_m())),
                yaw,
            );
            assert_eq!(delta.translation.length(), 0.0);
        }
    }

    #[test]
    #[should_panic]
    fn parallel_survey_axes_are_invalid() {
        MountAxes::new(MountDirection::Forward, MountDirection::Backward);
    }

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
        assert_eq!(RobotPoint::ORIGIN.forward_wheel_offset_m(), 0.0);
    }

    #[test]
    fn beam_bearings_and_invalid_coordinates_keep_explicit_semantics() {
        let point = RobotPoint::ORIGIN;
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
