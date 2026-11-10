//! Field and robot geometry in meters and radians.

use core::f64::consts::{PI, TAU};

use glam::DVec2;

pub const INCH: f64 = 0.0254;

/// Counterclockwise heading from field +X, wrapped to [-π, π).
pub fn wrap_heading(radians: f64) -> f64 {
    let wrapped = (radians + PI) % TAU;
    if wrapped < 0.0 {
        wrapped + PI
    } else {
        wrapped - PI
    }
}

/// Converts clockwise compass degrees from field +Y to a mathematical heading.
pub fn compass_heading(degrees: f64) -> f64 {
    wrap_heading(PI / 2.0 - degrees.to_radians())
}

pub fn direction(heading: f64) -> DVec2 {
    DVec2::new(libm::cos(heading), libm::sin(heading))
}

/// Field-centered pose. The robot frame has +X forward and +Y left.
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
        self.position + direction(self.heading).rotate(point)
    }

    pub fn to_local(self, point: DVec2) -> DVec2 {
        direction(-self.heading).rotate(point - self.position)
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
            (libm::sin(angle) / angle, (1.0 - libm::cos(angle)) / angle)
        };
        let local = DVec2::new(sinc, cosc).rotate(delta.translation);
        Self {
            position: self.transform_point(local),
            heading: wrap_heading(self.heading + angle),
        }
    }
}

/// Integrated forward, lateral, and angular travel over one sample.
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
        2.0 * target.y / distance_squared
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quarter_circle_and_compass() {
        let pose = Pose::default().integrate(BodyDelta {
            translation: DVec2::new(PI / 2.0, 0.0),
            heading: PI / 2.0,
        });
        assert!(pose.position.distance(DVec2::ONE) < 1e-10);
        assert_eq!(pose.heading, compass_heading(0.0));
        assert_eq!(compass_heading(90.0), 0.0);
    }
}
