//! Field and robot geometry in meters and radians.

use core::f64::consts::{PI, TAU};

pub use glam::DVec2;
use uom::si::{
    f64::{Angle, AngularVelocity, Length, Velocity},
    velocity::meter_per_second,
};

/// Counterclockwise heading from +Y forward, wrapped to [-π, π).
pub fn wrap_heading(angle: Angle) -> Angle {
    let radians = angle.value;
    if (-PI..PI).contains(&radians) {
        return angle;
    }
    let wrapped = (radians + PI) % TAU;
    crate::qty!((if wrapped < 0.0 {
            wrapped + PI
        } else {
            wrapped - PI
        }) rad)
}

/// Converts clockwise compass degrees from field +Y to a mathematical heading.
pub fn compass_heading(clockwise: Angle) -> Angle {
    wrap_heading(-clockwise)
}

/// Forward unit vector at a heading: 0 faces +Y, +π/2 faces -X (left).
pub fn direction(heading: Angle) -> DVec2 {
    let (sin, cos) = libm::sincos(heading.value);
    DVec2::new(-sin, cos)
}

/// Heading toward a right/forward displacement. Zero displacement yields zero.
pub fn bearing(displacement: DVec2) -> Angle {
    crate::qty!((libm::atan2(-displacement.x, displacement.y)) rad)
}

/// Converts unit-bearing right/forward coordinates into a position in meters.
pub const fn point(right: Length, forward: Length) -> DVec2 {
    DVec2::new(right.value, forward.value)
}

/// Mathematical rotation operator, distinct from a robot's forward direction.
pub fn rotation(angle: Angle) -> DVec2 {
    let (sin, cos) = libm::sincos(angle.value);
    DVec2::new(cos, sin)
}

/// Pose of the chassis center of rotation in a fixed surveyed field frame.
/// The Origin is that point in the chassis frame. Heading zero faces field +Y;
/// positive yaw turns left about +Z. Length is metres, heading radians.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Pose {
    pub position: DVec2,
    pub heading: Angle,
}

impl Pose {
    /// Creates a pose from a right/forward position in meters and a CCW heading.
    pub const fn new(position: DVec2, heading: Angle) -> Self {
        Self { position, heading }
    }

    pub fn is_finite(self) -> bool {
        self.position.is_finite() && self.heading.value.is_finite()
    }

    pub fn transform_point(self, point: DVec2) -> DVec2 {
        self.position + rotation(self.heading).rotate(point)
    }

    pub fn to_local(self, point: DVec2) -> DVec2 {
        rotation(-self.heading).rotate(point - self.position)
    }

    /// Interpolate positions and follow the shortest angular displacement.
    pub fn interpolate(self, end: Self, fraction: f64) -> Self {
        let delta = wrap_heading(end.heading - self.heading);
        let heading = wrap_heading(self.heading + delta * fraction);
        Self::new(self.position.lerp(end.position, fraction), heading)
    }

    /// Expresses this field pose relative to a surveyed origin and its initial
    /// right/forward orientation. Units and positive yaw convention are unchanged.
    pub fn in_frame(self, origin: Pose) -> Self {
        Self {
            position: origin.to_local(self.position),
            heading: wrap_heading(self.heading - origin.heading),
        }
    }

    /// Integrates a constant body twist over one sample using the SE(2) exponential.
    pub fn integrate(self, delta: BodyDelta) -> Self {
        let angle = delta.heading.value;
        let (sinc, cosc) = if angle.abs() < 1e-5 {
            (
                1.0 - angle * angle / 6.0,
                angle * (0.5 - angle * angle / 24.0),
            )
        } else {
            let (sin, cos) = libm::sincos(angle);
            (sin / angle, (1.0 - cos) / angle)
        };
        let local = DVec2::new(sinc, cosc).rotate(delta.translation);
        Self {
            position: self.transform_point(local),
            heading: wrap_heading(self.heading + delta.heading),
        }
    }
}

/// Integrated right (+X), forward (+Y), and CCW angular travel over one sample.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct BodyDelta {
    pub translation: DVec2,
    pub heading: Angle,
}

impl BodyDelta {
    pub fn is_finite(self) -> bool {
        self.translation.is_finite() && self.heading.value.is_finite()
    }
}

/// Converts body velocities to left and right wheel velocities, in meters per second.
pub fn wheel_speeds(
    linear: Velocity,
    angular: AngularVelocity,
    track_width: Length,
) -> [Velocity; 2] {
    let turn: Velocity = Velocity::new::<meter_per_second>(angular.value * track_width.value * 0.5);
    [linear - turn, linear + turn]
}

/// Intersections along a directed segment, with each root bounded to [0, 1].
pub fn circle_segment_fractions(
    center: DVec2,
    radius: Length,
    start: DVec2,
    end: DVec2,
) -> [Option<f64>; 2] {
    let radius = radius.value;
    let offset = start - center;
    let segment = end - start;
    let length_squared = segment.length_squared();
    let projection = offset.dot(segment);
    let discriminant =
        projection * projection - length_squared * (offset.length_squared() - radius * radius);
    if length_squared <= 0.0 || discriminant < 0.0 {
        return [None; 2];
    }
    let root = libm::sqrt(discriminant);
    [-root, root].map(|root| {
        let fraction = (-projection + root) / length_squared;
        (0.0..=1.0).contains(&fraction).then_some(fraction)
    })
}

/// Signed curvature of the tangent arc from robot origin to a local target.
pub fn pursuit_curvature(target: DVec2) -> f64 {
    let distance_squared = target.length_squared();
    if distance_squared > 1e-12 {
        -2.0 * target.x / distance_squared
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn field_pose_registration_uses_initial_origin_right_forward_axes() {
        let origin = Pose::new(
            crate::geometry::point(crate::qty!(2.0 m), crate::qty!(3.0 m)),
            crate::qty!((PI / 2.0) rad),
        );
        assert!(origin.in_frame(origin).position.length() < 1e-12);
        assert_eq!(origin.in_frame(origin).heading.value, 0.0);
        let right = Pose::new(
            crate::geometry::point(crate::qty!(2.0 m), crate::qty!(4.0 m)),
            crate::qty!((PI / 2.0) rad),
        )
        .in_frame(origin);
        assert!(right.position.distance(DVec2::X) < 1e-12);
        let forward = Pose::new(
            crate::geometry::point(crate::qty!(1.0 m), crate::qty!(3.0 m)),
            crate::qty!((PI) rad),
        )
        .in_frame(origin);
        assert!(forward.position.distance(DVec2::Y) < 1e-12);
        assert!((forward.heading.value - PI / 2.0).abs() < 1e-12);
    }
    #[test]
    fn canonical_axes_rotation_and_compass() {
        assert_eq!(Pose::default().transform_point(DVec2::X), DVec2::X);
        assert_eq!(Pose::default().transform_point(DVec2::Y), DVec2::Y);
        assert!(direction(crate::qty!(0.0 rad)).distance(DVec2::Y) < 1e-12);
        assert!(direction(crate::qty!((PI / 2.0) rad)).distance(-DVec2::X) < 1e-12);
        assert!(direction(crate::qty!((-PI / 2.0) rad)).distance(DVec2::X) < 1e-12);
        let pose = Pose::default().integrate(BodyDelta {
            translation: DVec2::new(0.0, PI / 2.0),
            heading: crate::qty!((PI / 2.0) rad),
        });
        assert!(pose.position.distance(DVec2::new(-1.0, 1.0)) < 1e-10);
        assert_eq!(compass_heading(crate::qty!(0.0 rad)), crate::qty!(0.0 rad));
        assert_eq!(
            compass_heading(crate::qty!(90.0 deg)),
            crate::qty!((-PI / 2.0) rad)
        );
        for heading in [0.0, PI / 2.0, -PI / 2.0, 2.1] {
            assert!(
                wrap_heading(crate::qty!(((bearing(direction(crate::angle!(heading, rad))) - crate::angle!(heading, rad))
                        .value) rad))
                .value
                .abs()
                    < 1e-12
            );
            let pose = Pose::new(
                crate::geometry::point(crate::qty!(0.2 m), crate::qty!(-0.4 m)),
                crate::qty!((heading) rad),
            );
            let local = DVec2::new(0.3, 0.7);
            assert!(pose.to_local(pose.transform_point(local)).distance(local) < 1e-12);
        }
    }
}
#[test]
fn pose_interpolation_crosses_the_heading_wrap_by_the_shortest_arc() {
    let start = Pose::new(DVec2::ZERO, crate::qty!(170.0 deg));
    let end = Pose::new(DVec2::new(2.0, 4.0), crate::qty!(-170.0 deg));
    let middle = start.interpolate(end, 0.5);
    assert_eq!(middle.position, DVec2::new(1.0, 2.0));
    assert!((middle.heading.value.abs() - PI).abs() < 1e-12);
}
