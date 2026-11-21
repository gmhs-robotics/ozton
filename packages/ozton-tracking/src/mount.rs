//! Installation coordinates measured from the chassis center of rotation.
use ozton_math::{
    localization::range::Mount,
    uom::{
        ConstZero,
        si::f64::{Angle, Length},
    },
};

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
            right: Length {
                value: x.value * x_right + y.value * y_right,
                ..Length::ZERO
            },
            forward: Length {
                value: x.value * x_forward + y.value * y_forward,
                ..Length::ZERO
            },
            up,
        }
    }
}

/// Three coordinates in the right/forward/up chassis frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec3<T> {
    pub right: T,
    pub forward: T,
    pub up: T,
}

impl<T> Vec3<T> {
    pub const fn new(right: T, forward: T, up: T) -> Self {
        Self { right, forward, up }
    }
}

/// Sensor or wheel location relative to the chassis center of rotation.
pub type RobotPoint = Vec3<Length>;

impl RobotPoint {
    /// The Origin, at the chassis center of rotation in the robot frame.
    pub const ORIGIN: Self = Self {
        right: Length::ZERO,
        forward: Length::ZERO,
        up: Length::ZERO,
    };

    /// Validates coordinates before a static mount is used.
    pub const fn is_finite(self) -> bool {
        self.right.value.is_finite() && self.forward.value.is_finite() && self.up.value.is_finite()
    }

    /// Adapts a horizontal distance beam to the canonical right/forward planar frame.
    /// Zero bearing faces forward; positive bearing turns left about robot +Z.
    /// Height remains in this point; tilted beams require a separate 3D model.
    pub fn horizontal_range_mount(self, bearing_ccw_from_forward: Angle) -> Option<Mount> {
        if ![
            self.right.value,
            self.forward.value,
            self.up.value,
            bearing_ccw_from_forward.value,
        ]
        .iter()
        .all(|v| v.is_finite())
        {
            return None;
        }
        Some(Mount::new(
            self.right,
            self.forward,
            bearing_ccw_from_forward,
        ))
    }
}

#[cfg(test)]
mod tests {
    use ozton_math::uom::si::{angle::radian, length::meter};

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
            assert_eq!(
                axes.point(
                    Length::new::<meter>(0.0),
                    Length::new::<meter>(0.0),
                    Length::new::<meter>(0.0)
                ),
                RobotPoint::ORIGIN
            );
            let point = axes.point(
                Length::new::<meter>(x),
                Length::new::<meter>(y),
                Length::new::<meter>(0.3),
            );
            assert_eq!(
                point,
                RobotPoint::new(
                    Length::new::<meter>(-0.1),
                    Length::new::<meter>(0.2),
                    Length::new::<meter>(0.3)
                )
            );

            let yaw = 0.4;
            let delta = ozton_math::localization::odometry::body_delta(
                (Length::new::<meter>(-0.1 * yaw), point.right),
                Some((Length::new::<meter>(-0.2 * yaw), point.forward)),
                Angle::new::<radian>(yaw),
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
        let point = RobotPoint::new(
            Length::new::<meter>(-0.1),
            Length::new::<meter>(0.2),
            Length::new::<meter>(0.3),
        );
        let mount = point
            .horizontal_range_mount(Angle::new::<radian>(0.0))
            .unwrap();
        assert_eq!(mount.offset.x, -0.1);
        assert_eq!(mount.offset.y, 0.2);
        assert_eq!(mount.heading, 0.0);
        assert_eq!(point.up, Length::new::<meter>(0.3));
        assert_eq!(point.right.value, -0.1);
        assert_eq!(point.forward.value, 0.2);
        assert_eq!(RobotPoint::ORIGIN.right.value, 0.0);
    }

    #[test]
    fn beam_bearings_and_invalid_coordinates_keep_explicit_semantics() {
        let point = RobotPoint::ORIGIN;
        assert_eq!(
            point
                .horizontal_range_mount(Angle::new::<radian>(-core::f64::consts::FRAC_PI_2))
                .unwrap()
                .heading,
            -core::f64::consts::FRAC_PI_2
        );
        assert!(
            point
                .horizontal_range_mount(Angle::new::<radian>(f64::NAN))
                .is_none()
        );
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(
                !RobotPoint::new(
                    Length::new::<meter>(invalid),
                    Length::new::<meter>(0.0),
                    Length::new::<meter>(0.0)
                )
                .is_finite()
            );
            assert!(
                !RobotPoint::new(
                    Length::new::<meter>(0.0),
                    Length::new::<meter>(invalid),
                    Length::new::<meter>(0.0)
                )
                .is_finite()
            );
            assert!(
                !RobotPoint::new(
                    Length::new::<meter>(0.0),
                    Length::new::<meter>(0.0),
                    Length::new::<meter>(invalid)
                )
                .is_finite()
            );
        }
    }
}
