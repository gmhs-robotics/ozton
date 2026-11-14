//! Field and robot geometry in meters and radians.

use core::f64::consts::{PI, TAU};

use glam::DVec2;

pub const INCH: f64 = 0.0254;

/// Counterclockwise heading from +Y forward, wrapped to [-π, π).
pub fn wrap_heading(radians: f64) -> f64 {
    if (-PI..PI).contains(&radians) {
        return radians;
    }
    let wrapped = (radians + PI) % TAU;
    if wrapped < 0.0 {
        wrapped + PI
    } else {
        wrapped - PI
    }
}

/// Converts clockwise compass degrees from field +Y to a mathematical heading.
pub fn compass_heading(degrees: f64) -> f64 {
    wrap_heading(-degrees.to_radians())
}

/// Forward unit vector at a heading: 0 faces +Y, +π/2 faces -X (left).
pub fn direction(heading: f64) -> DVec2 {
    let (sin, cos) = libm::sincos(heading);
    DVec2::new(-sin, cos)
}

/// Heading toward a right/forward displacement. Zero displacement yields zero.
pub fn bearing(displacement: DVec2) -> f64 {
    libm::atan2(-displacement.x, displacement.y)
}

/// Mathematical rotation operator, distinct from a robot's forward direction.
pub fn rotation(angle: f64) -> DVec2 {
    let (sin, cos) = libm::sincos(angle);
    DVec2::new(cos, sin)
}

/// CC pose in a fixed surveyed field frame: +X right, +Y forward, +Z up.
/// Body points use the same axes relative to current CC. Heading zero faces +Y;
/// positive yaw turns left about +Z. Length is metres, heading radians.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Pose {
    pub position: DVec2,
    pub heading: f64,
}

impl Pose {
    pub const fn new(x: f64, y: f64, heading: f64) -> Self {
        Self {
            position: DVec2::new(x, y),
            heading,
        }
    }

    pub fn is_finite(self) -> bool {
        self.position.is_finite() && self.heading.is_finite()
    }

    pub fn transform_point(self, point: DVec2) -> DVec2 {
        self.position + rotation(self.heading).rotate(point)
    }

    pub fn to_local(self, point: DVec2) -> DVec2 {
        rotation(-self.heading).rotate(point - self.position)
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
        let angle = delta.heading;
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
            heading: wrap_heading(self.heading + angle),
        }
    }
}

/// Integrated right (+X), forward (+Y), and CCW angular travel over one sample.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct BodyDelta {
    pub translation: DVec2,
    pub heading: f64,
}

impl BodyDelta {
    pub fn is_finite(self) -> bool {
        self.translation.is_finite() && self.heading.is_finite()
    }
}

/// Converts body velocities to left and right wheel velocities, in meters per second.
pub fn wheel_speeds(linear: f64, angular: f64, track_width: f64) -> [f64; 2] {
    let turn = angular * track_width / 2.0;
    [linear - turn, linear + turn]
}

/// Intersections along a directed segment, with each root bounded to [0, 1].
pub fn circle_segment_fractions(
    center: DVec2,
    radius: f64,
    start: DVec2,
    end: DVec2,
) -> [Option<f64>; 2] {
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
    fn heading_wrap_preserves_boundaries_and_non_finite_rejection() {
        for angle in [-1e6, -TAU, -PI, -0.1, 0.0, 0.1, PI, TAU, 1e6] {
            let wrapped = wrap_heading(angle);
            assert!((-PI..PI).contains(&wrapped));
            assert!((libm::sin(wrapped) - libm::sin(angle)).abs() < 1e-10);
            assert!((libm::cos(wrapped) - libm::cos(angle)).abs() < 1e-10);
        }
        assert_eq!(wrap_heading(PI), -PI);
        assert_eq!(wrap_heading(-PI), -PI);
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(wrap_heading(invalid).is_nan());
        }
    }

    #[test]
    fn small_and_large_twists_match_independent_arc_solution() {
        for yaw in [-2.9, -0.4, -1e-5, -1e-8, 0.0, 1e-8, 1e-5, 0.4, 2.9] {
            for heading in [-PI, -1.2, 0.0, 1.2, PI] {
                let start = Pose::new(0.3, -0.8, heading);
                let delta = BodyDelta {
                    translation: DVec2::new(0.07, 0.13),
                    heading: yaw,
                };
                let half = yaw * 0.5;
                let scale = if yaw == 0.0 { 1.0 } else { half.sin() / half };
                let mid = heading + half;
                let local = delta.translation * scale;
                let expected = start.position
                    + DVec2::new(
                        mid.cos() * local.x - mid.sin() * local.y,
                        mid.sin() * local.x + mid.cos() * local.y,
                    );
                assert!(start.integrate(delta).position.distance(expected) < 1e-12);
            }
        }
    }

    #[test]
    fn field_pose_registration_uses_initial_cc_right_forward_axes() {
        let origin = Pose::new(2.0, 3.0, PI / 2.0);
        assert!(origin.in_frame(origin).position.length() < 1e-12);
        assert_eq!(origin.in_frame(origin).heading, 0.0);
        let right = Pose::new(2.0, 4.0, PI / 2.0).in_frame(origin);
        assert!(right.position.distance(DVec2::X) < 1e-12);
        let forward = Pose::new(1.0, 3.0, PI).in_frame(origin);
        assert!(forward.position.distance(DVec2::Y) < 1e-12);
        assert!((forward.heading - PI / 2.0).abs() < 1e-12);
    }
    #[test]
    fn canonical_axes_rotation_and_compass() {
        assert_eq!(Pose::default().transform_point(DVec2::X), DVec2::X);
        assert_eq!(Pose::default().transform_point(DVec2::Y), DVec2::Y);
        assert!(direction(0.0).distance(DVec2::Y) < 1e-12);
        assert!(direction(PI / 2.0).distance(-DVec2::X) < 1e-12);
        assert!(direction(-PI / 2.0).distance(DVec2::X) < 1e-12);
        let pose = Pose::default().integrate(BodyDelta {
            translation: DVec2::new(0.0, PI / 2.0),
            heading: PI / 2.0,
        });
        assert!(pose.position.distance(DVec2::new(-1.0, 1.0)) < 1e-10);
        assert_eq!(compass_heading(0.0), 0.0);
        assert_eq!(compass_heading(90.0), -PI / 2.0);
        for heading in [0.0, PI / 2.0, -PI / 2.0, 2.1] {
            assert!(wrap_heading(bearing(direction(heading)) - heading).abs() < 1e-12);
            let pose = Pose::new(0.2, -0.4, heading);
            let local = DVec2::new(0.3, 0.7);
            assert!(pose.to_local(pose.transform_point(local)).distance(local) < 1e-12);
        }
    }
}
