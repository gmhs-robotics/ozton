//! Installation coordinates measured from the chassis center of rotation.
pub use ozton_math::units::Vec3;
#[doc(hidden)]
pub use ozton_math::uom;
use uom::{ConstZero, si::f64::Length};

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
    pub const fn point(self, survey: Vec3<Length>) -> Vec3<Length> {
        let (x_right, x_forward) = self.x.right_forward();
        let (y_right, y_forward) = self.y.right_forward();
        Vec3 {
            right: Length {
                value: survey.right.value * x_right + survey.forward.value * y_right,
                ..Length::ZERO
            },
            forward: Length {
                value: survey.right.value * x_forward + survey.forward.value * y_forward,
                ..Length::ZERO
            },
            up: survey.up,
        }
    }
}

#[cfg(test)]
mod tests {
    use ozton_math::uom::si::{angle::radian, f64::Angle, length::meter};

    use super::*;
    use crate::position;

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
                axes.point(position!(0 m, 0 m, 0 m)),
                position!(0 m, 0 m, 0 m)
            );
            let point = axes.point(Vec3::new(
                Length::new::<meter>(x),
                Length::new::<meter>(y),
                Length::new::<meter>(0.3),
            ));
            assert_eq!(point, position!(-0.1 m, 0.2 m, 0.3 m));

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
    #[should_panic(expected = "x_right * y_right + x_forward * y_forward == 0.0")]
    fn parallel_survey_axes_are_invalid() {
        MountAxes::new(MountDirection::Forward, MountDirection::Backward);
    }

    #[test]
    fn range_mount_preserves_coordinates_and_bearing() {
        use crate::range::RangeSensorConfig;

        const MOUNT: RangeSensorConfig = RangeSensorConfig::at_mount(
            position!(-2 in, 0.2 m, 30 cm),
            Angle {
                value: -core::f64::consts::FRAC_PI_2,
                ..Angle::ZERO
            },
        );
        assert_eq!(MOUNT.mount.position.x, -0.0508);
        assert_eq!(MOUNT.mount.position.y, 0.2);
        assert_eq!(MOUNT.mount_height.value, 0.3);
        assert_eq!(MOUNT.mount.heading.value, -core::f64::consts::FRAC_PI_2);
    }

    #[test]
    fn invalid_coordinates_are_rejected_on_each_axis() {
        assert!(position!(0 m, 0 m, 0 m).is_finite());
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let invalid = Length::new::<meter>(invalid);
            for point in [
                Vec3::new(invalid, Length::ZERO, Length::ZERO),
                Vec3::new(Length::ZERO, invalid, Length::ZERO),
                Vec3::new(Length::ZERO, Length::ZERO, invalid),
            ] {
                assert!(!point.is_finite());
            }
        }
    }
}
