//! Installation coordinates measured from the chassis center of rotation.
pub use ozton_math::units::Vec3;
#[doc(hidden)]
pub use ozton_math::uom;
use ozton_math::{geometry::DVec2, path::clearance::Footprint};
#[cfg(test)]
use uom::ConstZero;
use uom::si::f64::{Angle, Length};
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
            right: ozton_math::qty!((survey.right.value * x_right + survey.forward.value * y_right) m),
            forward: ozton_math::qty!((survey.right.value * x_forward + survey.forward.value * y_forward) m),
            up: survey.up,
        }
    }
}

/// A fixed translation from surveyed right/forward/up CAD coordinates to
/// the robot's local origin. Configure once, before building tracking or
/// localization. A differential drivetrain's origin should lie on its
/// effective axle, midway between the left and right rolling centerlines.
///
/// Mount bearings and field coordinates are unchanged by this translation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MountFrame {
    origin: Vec3<Length>,
}

impl Default for MountFrame {
    fn default() -> Self {
        Self::new()
    }
}

impl MountFrame {
    pub const fn new() -> Self {
        Self {
            origin: ozton_math::position_xyz!(0 m, 0 m, 0 m),
        }
    }

    /// The position of the new origin in the original survey.
    #[must_use]
    pub const fn with_origin(mut self, origin: Vec3<Length>) -> Self {
        self.origin = origin;
        self
    }

    pub const fn origin(self) -> Vec3<Length> {
        self.origin
    }

    /// Convert a surveyed point (sensor, center of mass, etc.) to local coordinates.
    pub const fn point(self, position: Vec3<Length>) -> Vec3<Length> {
        Vec3::new(
            ozton_math::qty!((position.right.value - self.origin.right.value) m),
            ozton_math::qty!((position.forward.value - self.origin.forward.value) m),
            ozton_math::qty!((position.up.value - self.origin.up.value) m),
        )
    }

    /// Convert a surveyed sensor mount without changing its bearing.
    pub const fn mount(self, (position, bearing): (Vec3<Length>, Angle)) -> (Vec3<Length>, Angle) {
        (self.point(position), bearing)
    }

    /// Rebase the physical footprint from the same survey. Field-bounded
    /// motion requires the local origin to remain on the floor.
    pub fn footprint(self, footprint: Footprint) -> Option<Footprint> {
        if !self.origin.is_finite() || self.origin.up.value != 0.0 {
            return None;
        }
        footprint.with_reference_offset(DVec2::new(
            self.origin.right.value,
            self.origin.forward.value,
        ))
    }
}

/// Estimate the longitudinal effective pivot in the same coordinate frame
/// as a sideways tracker, using signed rightward travel and unwrapped CCW yaw.
/// Use a substantial isolated turn in both directions; this estimates an
/// effective kinematic parameter, not the physical wheel mounting point.
pub fn estimate_forward_pivot(
    tracker_forward: Length,
    travel: Length,
    yaw: Angle,
) -> Option<Length> {
    if !tracker_forward.value.is_finite()
        || !travel.value.is_finite()
        || !yaw.value.is_finite()
        || yaw.value.abs() < 1e-6
    {
        return None;
    }
    let pivot = tracker_forward.value + travel.value / yaw.value;
    pivot.is_finite().then(|| ozton_math::qty!((pivot) m))
}

#[cfg(test)]
mod tests {

    use ozton_math::position_xyz;

    use super::*;

    #[test]
    fn rebasing_preserves_world_sensor_and_footprint_geometry() {
        use ozton_math::geometry::Pose;

        let frame = MountFrame::new().with_origin(position_xyz!(0 m, -0.04 m, 0 m));
        let sensor = position_xyz!(0.02 m, -0.01 m, 0.08 m);
        let (local, bearing) = frame.mount((sensor, ozton_math::qty!(-90 deg)));
        assert!((local.forward.value - 0.03).abs() < 1e-12);
        assert_eq!(local.up, sensor.up);
        assert_eq!(bearing, ozton_math::qty!(-90 deg));
        assert_eq!(frame.point(frame.origin()), position_xyz!(0 m, 0 m, 0 m));

        let footprint = Footprint::from_extents(
            ozton_math::qty!(0.2 m),
            ozton_math::qty!(0.2 m),
            ozton_math::qty!(0.3 m),
            ozton_math::qty!(0.25 m),
        )
        .unwrap();
        let shifted = frame.footprint(footprint).unwrap();
        let offset = DVec2::new(0.0, frame.origin().forward.value);
        for yaw in [0.0, 0.7, -1.5, 3.0] {
            let survey = Pose::new(DVec2::new(1.0, 2.0), ozton_math::qty!((yaw) rad));
            let pivot = Pose::new(survey.transform_point(offset), survey.heading);
            let before =
                survey.transform_point(DVec2::new(sensor.right.value, sensor.forward.value));
            let after = pivot.transform_point(DVec2::new(local.right.value, local.forward.value));
            assert!(before.distance(after) < 1e-12);
            let (old_min, old_max) = footprint.bounds(survey);
            let (new_min, new_max) = shifted.bounds(pivot);
            assert!(old_min.distance(new_min) < 1e-12);
            assert!(old_max.distance(new_max) < 1e-12);
        }
        assert!(
            MountFrame::new()
                .with_origin(position_xyz!(0 m, 0 m, 0.1 m))
                .footprint(footprint)
                .is_none()
        );
    }

    #[test]
    fn bidirectional_turn_recovers_effective_pivot() {
        let tracker = ozton_math::qty!(-0.004 m);
        let expected = -0.037;
        for yaw in [-core::f64::consts::TAU, core::f64::consts::TAU] {
            let travel = (expected - tracker.value) * yaw;
            let pivot = estimate_forward_pivot(
                tracker,
                ozton_math::qty!((travel) m),
                ozton_math::qty!((yaw) rad),
            )
            .unwrap();
            assert!((pivot.value - expected).abs() < 1e-12);
            let local = tracker - pivot;
            let result = estimate_forward_pivot(
                local,
                ozton_math::qty!((travel) m),
                ozton_math::qty!((yaw) rad),
            )
            .unwrap();
            assert!(result.value.abs() < 1e-12);
        }
        assert!(estimate_forward_pivot(tracker, Length::ZERO, Angle::default()).is_none());
    }

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
                axes.point(position_xyz!(0 m, 0 m, 0 m)),
                position_xyz!(0 m, 0 m, 0 m)
            );
            let point = axes.point(Vec3::new(
                ozton_math::qty!((x) m),
                ozton_math::qty!((y) m),
                ozton_math::qty!(0.3 m),
            ));
            assert_eq!(point, position_xyz!(-0.1 m, 0.2 m, 0.3 m));

            let yaw = 0.4;
            let delta = ozton_math::localization::odometry::body_delta(
                (ozton_math::qty!((-0.1 * yaw) m), point.right),
                Some((ozton_math::qty!((-0.2 * yaw) m), point.forward)),
                ozton_math::qty!((yaw) rad),
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

        const MOUNT: RangeSensorConfig =
            RangeSensorConfig::at_mount(ozton_math::position_xyzh!(-2 in, 0.2 m, 30 cm, -90 deg));
        assert_eq!(MOUNT.mount.position.x, -0.0508);
        assert_eq!(MOUNT.mount.position.y, 0.2);
        assert_eq!(MOUNT.mount_height.value, 0.3);
        assert_eq!(MOUNT.mount.heading.value, -core::f64::consts::FRAC_PI_2);
    }

    #[test]
    fn invalid_coordinates_are_rejected_on_each_axis() {
        assert!(position_xyz!(0 m, 0 m, 0 m).is_finite());
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let invalid = ozton_math::qty!((invalid) m);
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
