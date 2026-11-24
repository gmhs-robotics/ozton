//! Departures and turning clearance inside a centered rectangular perimeter.

use glam::DVec2;
use uom::si::f64::{Angle, Length};

use super::{Footprint, forward_arc_end};
use crate::geometry::{Pose, direction};

mod approach;
pub use approach::{PointDeparture, approach_heading};

#[derive(Clone, Copy, Debug)]
pub struct RectangularClearance {
    footprint: Footprint,
    usable: DVec2,
    margin: f64,
    obstacles: [Option<(DVec2, DVec2)>; 4],
}

#[derive(Clone, Copy, Debug)]
pub struct DepartureArc {
    pub radius: Length,
    pub sweep: Angle,
}

impl RectangularClearance {
    pub fn new(footprint: Footprint, half_extents: DVec2, margin: Length) -> Option<Self> {
        let usable = half_extents - DVec2::splat(margin.value);
        (usable.is_finite() && usable.min_element() > 0.0 && margin.value >= 0.0)
            .then_some(Self {
                footprint,
                usable,
                margin: margin.value,
                obstacles: [None; 4],
            })
    }

    /// Add a solid rectangle to the clearance map, independently of range surfaces.
    pub fn with_obstacle(mut self, min: DVec2, max: DVec2) -> Option<Self> {
        if !min.is_finite() || !max.is_finite() || !min.cmplt(max).all() {
            return None;
        }
        *self.obstacles.iter_mut().find(|slot| slot.is_none())? = Some((
            min - DVec2::splat(self.margin),
            max + DVec2::splat(self.margin),
        ));
        Some(self)
    }

    fn obstacles_clear(self, min: DVec2, max: DVec2) -> bool {
        self.obstacles.iter().flatten().all(|&(a, b)| {
            max.cmplt(a).any() || min.cmpgt(b).any()
        })
    }

    /// Conservative swept AABB for constant-heading translation.
    pub fn contains_translation(self, start: Pose, end: DVec2) -> bool {
        let (min, max) = self.footprint.bounds(start);
        let delta = end - start.position;
        self.contains_aabb(min.min(min + delta), max.max(max + delta))
    }

    /// Conservative obstacle-only braking envelope using measured travel and yaw.
    pub fn obstacle_stop_clear(self, pose: Pose, translation: DVec2, yaw: Angle) -> bool {
        if self.obstacles.iter().all(Option::is_none) {
            return true;
        }
        if !pose.is_finite() || !translation.is_finite() || !yaw.value.is_finite() {
            return false;
        }
        let (min, max) = self.footprint.bounds(pose);
        let pad = DVec2::splat(self.footprint.turn_radius() * yaw.value.abs().min(2.0));
        self.obstacles_clear(
            min.min(min + translation) - pad,
            max.max(max + translation) + pad,
        )
    }

    /// Check the oriented footprint, not the full turning disk.
    pub fn contains_pose(self, pose: Pose) -> bool {
        if !pose.is_finite() {
            return false;
        }
        let (min, max) = self.footprint.bounds(pose);
        self.contains_bounds(min, max)
    }

    /// Conservative bound for the swept footprint over interpolated poses.
    /// Every corner moves at most center travel plus radius times heading change.
    pub fn contains_segment(self, a: Pose, b: Pose) -> bool {
        let travel = a.position.distance(b.position);
        let turn = crate::geometry::wrap_heading(b.heading - a.heading)
            .value
            .abs();
        let pad = (travel + self.footprint.turn_radius() * turn) * 0.5;
        let (amin, amax) = self.footprint.bounds(a);
        let (bmin, bmax) = self.footprint.bounds(b);
        self.contains_bounds(
            amin.min(bmin) - DVec2::splat(pad),
            amax.max(bmax) + DVec2::splat(pad),
        )
    }

    pub fn can_turn(self, pose: Pose) -> bool {
        let extent = DVec2::splat(self.footprint.turn_radius());
        pose.is_finite()
            && pose.position.abs().cmple(self.turn_area(0.0)).all()
            && self.obstacles_clear(pose.position - extent, pose.position + extent)
    }

    fn turn_area(self, reserve: f64) -> DVec2 {
        self.usable - DVec2::splat(self.footprint.turn_radius() + reserve)
    }

    pub fn contains_aabb(self, min: DVec2, max: DVec2) -> bool {
        min.is_finite() && max.is_finite() && min.cmple(max).all() && self.contains_bounds(min, max)
    }

    fn contains_bounds(self, min: DVec2, max: DVec2) -> bool {
        min.cmpge(-self.usable).all()
            && max.cmple(self.usable).all()
            && self.obstacles_clear(min, max)
    }

    /// Distance to the first position with room to pivot, following the current heading.
    /// Both ends of the straight sweep must fit with the original chassis orientation.
    pub fn forward_to_turn(self, start: Pose, reserve: Length) -> Option<Length> {
        let turn_area = self.turn_area(reserve.value);
        if !start.is_finite()
            || !reserve.value.is_finite()
            || reserve.value < 0.0
            || turn_area.min_element() < 0.0
        {
            return None;
        }

        let forward = direction(start.heading);
        let (min, max) = self.footprint.bounds(start);
        if !self.contains_bounds(min, max) {
            return None;
        }

        let mut enter: f64 = 0.0;
        let mut leave = f64::INFINITY;
        for axis in 0..2 {
            let velocity = forward[axis];
            let origin = start.position[axis];
            let limit = turn_area[axis];
            if velocity.abs() < 1e-9 {
                if origin.abs() > limit {
                    return None;
                }
            } else {
                let near = (-limit - origin) / velocity;
                let far = (limit - origin) / velocity;
                enter = enter.max(near.min(far));
                leave = leave.min(near.max(far));
            }
        }
        // The straight segment lies inside a convex rectangle. Its two ends suffice.
        let displacement = enter * forward;
        (enter <= leave
            && self.contains_bounds(min.min(min + displacement), max.max(max + displacement))
            && self.can_turn(Pose::new(start.position + displacement, start.heading)))
            .then_some(crate::qty!((enter) m))
    }

    /// Try successively wider arcs in both directions, including a straight continuation.
    /// Minimize outer-wheel travel over the checked arcs and continuations.
    pub fn departure_arc(
        self,
        start: Pose,
        minimum_radius: Length,
        reserve: Length,
    ) -> Option<DepartureArc> {
        if !minimum_radius.value.is_finite() || minimum_radius.value < 0.0 {
            return None;
        }
        self.arcs(minimum_radius)
            .filter_map(|arc| {
                let end = forward_arc_end(start, arc.radius, arc.sweep);
                let straight = self.forward_to_turn(end, reserve)?;
                let (min, max) = self
                    .footprint
                    .forward_arc_bounds(start, arc.radius, arc.sweep)?;
                self.contains_bounds(min, max).then_some((
                    arc,
                    straight.value
                        + (arc.radius.value + minimum_radius.value) * arc.sweep.value.abs(),
                ))
            })
            .min_by(|(_, left), (_, right)| left.total_cmp(right))
            .map(|(arc, _)| arc)
    }
    fn arcs(self, minimum_radius: Length) -> impl Iterator<Item = DepartureArc> {
        let first_radius = minimum_radius.value.max(0.05);
        let radius_step = self.usable.max_element() / 10.0;
        (0..=40)
            .map(move |step| crate::qty!((first_radius + radius_step * f64::from(step)) m))
            .flat_map(|radius| {
                (-18..=18)
                    .filter(|step| *step != 0)
                    .map(move |step| DepartureArc {
                        radius,
                        sweep: crate::qty!((5.0 * f64::from(step)) deg),
                    })
            })
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn mirrored_obstacles_block_straight_crossings_but_allow_side_passes() {
        let chassis = Footprint::new(crate::qty!(15.1 in), crate::qty!(15 in)).unwrap();
        let field = RectangularClearance::new(
            chassis,
            crate::position_xy!(36.75 in, 54.5 in),
            crate::qty!(1 in),
        )
        .unwrap()
        .with_obstacle(
            crate::position_xy!(-2.5 in, 16.25 in),
            crate::position_xy!(1.5 in, 20.25 in),
        )
        .unwrap()
        .with_obstacle(
            crate::position_xy!(-1.5 in, -20.25 in),
            crate::position_xy!(2.5 in, -16.25 in),
        )
        .unwrap();
        assert!(field.can_turn(Pose::default()));
        for sign in [-1.0, 1.0] {
            assert!(!field.contains_translation(
                Pose::default(),
                crate::position_xy!(0 in, (sign * 36.0) in),
            ));
            let start = Pose::new(
                crate::position_xy!((sign * 20.0) in, -32 in),
                Angle::default(),
            );
            assert!(field.contains_translation(
                start,
                crate::position_xy!((sign * 20.0) in, 32 in),
            ));
        }
    }

    #[test]
    fn corner_departure_needs_space_and_reaches_a_turning_area() {
        let inches = |value| crate::qty!((value) in);
        let chassis = Footprint::from_extents(
            inches(7.55),
            inches(7.55),
            inches(7.761804),
            inches(7.238196),
        )
        .unwrap();
        let half_extents = crate::geometry::point(inches(19.0), inches(21.25));
        let field = RectangularClearance::new(chassis, half_extents, inches(0.5)).unwrap();
        let gap = inches(0.25);
        let (corner_from_cc, _) = chassis.bounds(Pose::default());
        let flush = Pose::new(-half_extents - corner_from_cc, Angle::default());
        let start = Pose::new(
            flush.position + DVec2::splat(inches(1.0).value),
            flush.heading,
        );
        let expected_cc = crate::geometry::point(inches(-10.45), inches(-12.488196));
        assert!((start.position - expected_cc).length() < 1e-12);
        assert!(field.forward_to_turn(start, gap).is_none());
        let arc = field.departure_arc(start, inches(6.5), gap).unwrap();
        assert!(arc.sweep.value < 0.0);
        let end = forward_arc_end(start, arc.radius, arc.sweep);
        let advance = field.forward_to_turn(end, gap).unwrap();
        assert!(advance > inches(0.0));
        let turn = Pose {
            position: end.position + direction(end.heading) * advance.value,
            ..end
        };
        assert!(field.can_turn(turn));

        assert!(field.forward_to_turn(flush, gap).is_none());
        assert!(field.departure_arc(flush, inches(6.5), gap).is_none());

        let field = RectangularClearance::new(chassis, half_extents, inches(0.0)).unwrap();
        let rear_sweep = Pose::new(
            crate::geometry::point(inches(0.0), inches(-13.6)),
            Angle::default(),
        );
        assert!(field.forward_to_turn(rear_sweep, gap).is_none());
        let clear_sweep = Pose::new(
            crate::geometry::point(inches(0.0), inches(-13.45)),
            Angle::default(),
        );
        assert!(field.forward_to_turn(clear_sweep, gap).is_some());
    }
}
