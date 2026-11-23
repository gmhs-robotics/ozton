//! Swept-footprint checks for arcs and in-place turns of a rectangular robot.

use core::f64::consts::TAU;

use glam::DVec2;
use uom::si::f64::{Angle, Length};

use crate::{
    geometry::{Pose, rotation},
    localization::range::Surface,
};

/// The horizontal footprint, centered on the drivetrain's turning center.
#[derive(Clone, Copy, Debug)]
pub struct Footprint {
    half_extents: DVec2,
}

impl Footprint {
    /// Width is along robot +X. Height is along robot +Y (the zero-degree axis).
    pub fn new(width: Length, height: Length) -> Option<Self> {
        let size = DVec2::new(width.value, height.value);
        (size.is_finite() && size.min_element() > 0.0).then_some(Self {
            half_extents: size * 0.5,
        })
    }

    /// Check the disk swept by the corners over a full turn.
    pub fn can_turn(self, pose: Pose, walls: &[Surface], clearance: Length) -> bool {
        let radius = self.half_extents.length() + clearance.value;
        pose.is_finite()
            && clearance.value.is_finite()
            && clearance.value >= 0.0
            && walls.iter().all(|wall| {
                let edge = wall.end - wall.start;
                let along = ((pose.position - wall.start).dot(edge) / edge.length_squared())
                    .clamp(0.0, 1.0);
                pose.position.distance_squared(wall.start + edge * along) > radius * radius
            })
    }

    /// Exact bounds of every corner over a constant-radius forward arc.
    /// The signed sweep follows the field convention, with positive angles turning left.
    pub fn forward_arc_bounds(
        self,
        start: Pose,
        radius: Length,
        sweep: Angle,
    ) -> Option<(DVec2, DVec2)> {
        if !start.is_finite()
            || !radius.value.is_finite()
            || radius.value <= 0.0
            || !sweep.value.is_finite()
            || sweep.value.abs() > TAU
        {
            return None;
        }
        let pivot = arc_pivot(start, radius, sweep);
        let end_rotation = rotation(sweep);
        let mut min = DVec2::splat(f64::INFINITY);
        let mut max = DVec2::splat(f64::NEG_INFINITY);
        let mut include = |point: DVec2| {
            min = min.min(point);
            max = max.max(point);
        };
        for x in [-self.half_extents.x, self.half_extents.x] {
            for y in [-self.half_extents.y, self.half_extents.y] {
                let corner = start.transform_point(DVec2::new(x, y));
                let spoke = corner - pivot;
                include(corner);
                include(pivot + end_rotation.rotate(spoke));
                for axis in [DVec2::X, DVec2::Y, DVec2::NEG_X, DVec2::NEG_Y] {
                    let angle = libm::atan2(spoke.perp_dot(axis), spoke.dot(axis));
                    let directed = (angle * sweep.value.signum() + TAU) % TAU;
                    if directed <= sweep.value.abs() {
                        include(pivot + axis * spoke.length());
                    }
                }
            }
        }
        Some((min, max))
    }
}

/// The center pose reached by driving a forward arc of the given radius and sweep.
pub fn forward_arc_end(start: Pose, radius: Length, sweep: Angle) -> Pose {
    let pivot = arc_pivot(start, radius, sweep);
    Pose {
        position: pivot + rotation(sweep).rotate(start.position - pivot),
        heading: start.heading + sweep.value,
    }
}

fn arc_pivot(start: Pose, radius: Length, sweep: Angle) -> DVec2 {
    start.transform_point(DVec2::new(-sweep.value.signum() * radius.value, 0.0))
}

#[cfg(test)]
mod tests {
    use uom::si::length::meter;

    use super::*;

    #[test]
    fn turn_requires_clearance_from_the_wall() {
        let m = Length::new::<meter>;
        let chassis = Footprint::new(m(0.4), m(0.5)).unwrap();
        let wall = Surface::new(m(0.2), m(-1.0), m(0.2), m(0.25));
        let start = Pose::default();
        assert!(!chassis.can_turn(start, &[wall], m(0.0)));
        let clear = Pose {
            position: DVec2::new(0.0, 0.6),
            ..start
        };
        assert!(chassis.can_turn(clear, &[wall], m(0.01)));
    }

    #[test]
    fn forward_arc_sweeps_into_the_wall_when_starting_flush() {
        use uom::si::{angle::degree, length::inch};

        let inches = Length::new::<inch>;
        let chassis = Footprint::new(inches(15.1), inches(15.0)).unwrap();
        let sweep = Angle::new::<degree>(25.0);
        let radius = inches(50.0);
        let left = inches(-19.0).value;
        let flush = Pose::new(inches(-19.0 + 15.1 / 2.0), inches(-12.75), Angle::default());
        let (min, _) = chassis
            .forward_arc_bounds(flush, radius, sweep * -1.0)
            .unwrap();
        assert!(min.x < left);

        let spaced = Pose {
            position: flush.position + DVec2::X * inches(1.0).value,
            ..flush
        };
        let (min, max) = chassis
            .forward_arc_bounds(spaced, radius, sweep * -1.0)
            .unwrap();
        assert!(min.x > left + inches(0.5).value);
        assert!(min.y > inches(-21.25).value);
        assert!(max.x < inches(19.0).value);
        assert!(max.y < inches(21.25).value);
        assert!(forward_arc_end(spaced, radius, sweep * -1.0).position.x > spaced.position.x);
    }
}
