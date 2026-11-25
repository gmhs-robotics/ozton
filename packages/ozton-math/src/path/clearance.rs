//! Swept-footprint checks for arcs and in-place turns of a rectangular robot.
mod polygon;
mod rectangle;
use core::f64::consts::TAU;

use glam::DVec2;
pub use polygon::{ConvexPolygon, MAX_OBSTACLES, MAX_POLYGON_VERTICES, Obstacles};
pub use rectangle::{AStar, DepartureArc, PointDeparture, RectangularClearance, approach_heading};
use uom::si::f64::{Angle, Length};

use crate::{
    geometry::{Pose, rotation},
    localization::range::Surface,
};

/// A surveyed physical wall. The caller must verify contact with the wall;
/// virtual exclusions and inferred near-wall poses are not contact observations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WallContact {
    Left,
    Right,
    Rear,
    Front,
}

impl WallContact {
    pub const fn axis(self) -> usize {
        match self {
            Self::Left | Self::Right => 0,
            Self::Rear | Self::Front => 1,
        }
    }

    /// Unit inward normal of the contacted wall.
    pub const fn inward(self) -> DVec2 {
        match self {
            Self::Left => DVec2::X,
            Self::Right => DVec2::NEG_X,
            Self::Rear => DVec2::Y,
            Self::Front => DVec2::NEG_Y,
        }
    }
}

/// The unilateral state constraint obtained from a verified flush placement
/// against a surveyed physical wall. Only the contacted plane is constrained;
/// virtual motion geometry is never included in this localization prior.
#[derive(Clone, Copy, Debug)]
pub struct WallContactConstraint {
    wall: WallContact,
    footprint: Footprint,
    plane: f64,
    tangential_sigma: Length,
}

impl WallContactConstraint {
    pub fn new(
        wall: WallContact,
        footprint: Footprint,
        half_extents: DVec2,
        pose: Pose,
        tangential_sigma: Length,
    ) -> Option<Self> {
        if !tangential_sigma.value.is_finite() || tangential_sigma.value < 0.0 {
            return None;
        }
        let field = RectangularClearance::new(footprint, half_extents, Length::default())?;
        let right = rotation(pose.heading);
        // A corner touching a wall is not evidence of flush alignment.
        if right.abs().min_element() > 8.0 * f64::EPSILON || !field.touches_wall(pose, wall) {
            return None;
        }
        Some(Self {
            wall,
            footprint,
            plane: -half_extents[wall.axis()],
            tangential_sigma,
        })
    }

    pub const fn wall(self) -> WallContact {
        self.wall
    }

    pub const fn tangential_sigma(self) -> Length {
        self.tangential_sigma
    }

    /// Orthogonal projection onto the physically feasible half-space at the
    /// measured heading. This conditions an estimate, never an actuator target.
    pub fn project(self, mut pose: Pose) -> Pose {
        let (min, max) = self.footprint.bounds(pose);
        let axis = self.wall.axis();
        let edge = if self.wall.inward()[axis] > 0.0 {
            min[axis]
        } else {
            -max[axis]
        };
        pose.position += self.wall.inward() * (self.plane - edge).max(0.0);
        pose
    }
}

/// A rectangular horizontal envelope measured from the center of rotation.
#[derive(Clone, Copy, Debug)]
pub struct Footprint {
    half_extents: DVec2,
    center_offset: DVec2,
}

impl Footprint {
    /// A footprint centered on the center of rotation, with width along X and length along Y.
    pub fn new(width: Length, height: Length) -> Option<Self> {
        let size = DVec2::new(width.value, height.value);
        (size.is_finite() && size.min_element() > 0.0).then_some(Self {
            half_extents: size * 0.5,
            center_offset: DVec2::ZERO,
        })
    }

    /// Distances from the center of rotation to the left, right, rear, and front edges.
    pub fn from_extents(left: Length, right: Length, rear: Length, front: Length) -> Option<Self> {
        let negative = DVec2::new(left.value, rear.value);
        let positive = DVec2::new(right.value, front.value);
        if !negative.is_finite()
            || !positive.is_finite()
            || negative.min_element() < 0.0
            || positive.min_element() < 0.0
        {
            return None;
        }
        let mut footprint = Self::new(left + right, rear + front)?;
        footprint.center_offset = (positive - negative) * 0.5;
        Some(footprint)
    }

    /// Express the same physical envelope relative to a different body-frame
    /// reference point, without changing its dimensions or world geometry.
    pub fn with_reference_offset(mut self, offset: DVec2) -> Option<Self> {
        if !offset.is_finite() {
            return None;
        }
        self.center_offset -= offset;
        self.center_offset.is_finite().then_some(self)
    }

    /// Field-aligned minimum and maximum corners in metres at the tracked pose.
    pub fn bounds(self, pose: Pose) -> (DVec2, DVec2) {
        let rotation = rotation(pose.heading);
        let center = pose.position + rotation.rotate(self.center_offset);
        let extent =
            rotation.abs() * self.half_extents.x + rotation.perp().abs() * self.half_extents.y;
        (center - extent, center + extent)
    }

    fn turn_radius(self) -> f64 {
        (self.half_extents + self.center_offset.abs()).length()
    }

    /// Check the disk swept by the corners over a full turn.
    pub fn can_turn(self, pose: Pose, walls: &[Surface], clearance: Length) -> bool {
        let radius = self.turn_radius() + clearance.value;
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
                let corner = start.transform_point(self.center_offset + DVec2::new(x, y));
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

/// The center-of-rotation pose after a forward arc of the given radius and sweep.
pub fn forward_arc_end(start: Pose, radius: Length, sweep: Angle) -> Pose {
    let pivot = arc_pivot(start, radius, sweep);
    Pose {
        position: pivot + rotation(sweep).rotate(start.position - pivot),
        heading: start.heading + sweep,
    }
}

fn arc_pivot(start: Pose, radius: Length, sweep: Angle) -> DVec2 {
    start.transform_point(DVec2::new(-sweep.value.signum() * radius.value, 0.0))
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn turn_requires_clearance_from_the_wall() {
        let m = |value| crate::qty!((value) m);
        let chassis = Footprint::new(m(0.4), m(0.5)).unwrap();
        let wall = Surface::new(
            crate::geometry::point(m(0.2), m(-1.0)),
            crate::geometry::point(m(0.2), m(0.25)),
            m(1.0),
        );
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
        let inches = |value| crate::qty!((value) in);
        let chassis = Footprint::new(inches(15.1), inches(15.0)).unwrap();
        let sweep = crate::qty!(25.0 deg);
        let radius = inches(50.0);
        let left = inches(-19.0).value;
        let flush = Pose::new(
            crate::geometry::point(inches(-19.0 + 15.1 / 2.0), inches(-12.75)),
            Angle::default(),
        );
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

    #[test]
    fn offset_footprint_rotates_and_sweeps_about_the_center_of_rotation() {
        let inches = |value| crate::qty!((value) in);
        let chassis = Footprint::from_extents(
            inches(7.55),
            inches(7.55),
            inches(7.761804),
            inches(7.238196),
        )
        .unwrap();
        let quarter_turn = crate::qty!(90.0 deg);
        let (min, max) = chassis.bounds(Pose::new(DVec2::ZERO, quarter_turn));
        assert!((min - crate::geometry::point(inches(-7.238196), inches(-7.55))).length() < 1e-12);
        assert!((max - crate::geometry::point(inches(7.761804), inches(7.55))).length() < 1e-12);

        let radius = inches(20.0);
        let (_, max) = chassis
            .forward_arc_bounds(Pose::default(), radius, quarter_turn)
            .unwrap();
        let outer_rear_corner = crate::geometry::point(radius + inches(7.55), inches(7.761804));
        assert!((max.x - (outer_rear_corner.length() - radius.value)).abs() < 1e-12);

        let wall = Surface::new(
            crate::geometry::point(inches(10.7), inches(-15.0)),
            crate::geometry::point(inches(10.7), inches(15.0)),
            inches(10.0),
        );
        assert!(!chassis.can_turn(Pose::default(), &[wall], inches(0.0)));
    }
}
