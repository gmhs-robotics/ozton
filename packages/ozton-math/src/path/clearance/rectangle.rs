//! Departures and turning clearance inside a centered rectangular perimeter.

use glam::DVec2;
use uom::si::f64::{Angle, Length};

use super::{Footprint, forward_arc_end};
use crate::geometry::{Pose, direction};

mod approach;
mod visibility;
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
        (usable.is_finite() && usable.min_element() > 0.0 && margin.value >= 0.0).then_some(Self {
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
        self.obstacles
            .iter()
            .flatten()
            .all(|&(a, b)| max.cmplt(a).any() || min.cmpgt(b).any())
    }

    /// Conservative swept AABB for constant-heading translation.
    pub fn contains_translation(self, start: Pose, end: DVec2) -> bool {
        let (min, max) = self.footprint.bounds(start);
        let delta = end - start.position;
        self.contains_aabb(min.min(min + delta), max.max(max + delta))
    }

    /// Conservative wall braking envelope, including the rotation of every
    /// chassis corner. This must run even when there are no motion obstacles.
    pub fn wall_stop_clear(self, pose: Pose, translation: DVec2, yaw: Angle) -> bool {
        if !pose.is_finite() || !translation.is_finite() || !yaw.value.is_finite() {
            return false;
        }
        let (min, max) = self.footprint.bounds(pose);
        let pad = DVec2::splat(self.footprint.turn_radius() * yaw.value.abs().min(2.0));
        (min.min(min + translation) - pad).cmpge(-self.usable).all()
            && (max.max(max + translation) + pad).cmple(self.usable).all()
    }

    /// Bound a braking sweep with independent signed travel intervals and a
    /// symmetric heading excursion. Each projected corner is bounded by the
    /// exact support of its circular arc. Each traveled increment is bounded
    /// by the extreme projection of its direction over the same heading range.
    /// This avoids charging corner rotation twice when already near a diagonal.
    pub fn braking_bounds(
        self,
        pose: Pose,
        forward: [f64; 2],
        lateral: [f64; 2],
        yaw: Angle,
    ) -> Option<(DVec2, DVec2)> {
        if !pose.is_finite()
            || !yaw.value.is_finite()
            || !forward.into_iter().chain(lateral).all(f64::is_finite)
            || forward[0] > forward[1]
            || lateral[0] > lateral[1]
        {
            return None;
        }
        let angle = yaw.value.abs().min(core::f64::consts::PI);
        let f = direction(pose.heading);
        let l = direction(pose.heading - crate::qty!(90 deg));
        let support = |axis: DVec2| {
            let mut corner_max = f64::NEG_INFINITY;
            for x in [-self.footprint.half_extents.x, self.footprint.half_extents.x] {
                for y in [-self.footprint.half_extents.y, self.footprint.half_extents.y] {
                    let corner = pose.transform_point(
                        self.footprint.center_offset + DVec2::new(x, y),
                    ) - pose.position;
                    corner_max = corner_max.max(rotated_support(corner, axis, angle));
                }
            }
            let travel_support = |d: DVec2, interval: [f64; 2]| {
                rotated_support(d * interval[0], axis, angle)
                    .max(rotated_support(d * interval[1], axis, angle))
                    .max(0.0)
            };
            corner_max + travel_support(f, forward) + travel_support(l, lateral)
        };
        let min = pose.position - DVec2::new(support(DVec2::NEG_X), support(DVec2::NEG_Y));
        let max = pose.position + DVec2::new(support(DVec2::X), support(DVec2::Y));
        Some((min, max))
    }

    /// Check the perimeter independently of fixed obstacles.
    pub fn wall_contains_aabb(self, min: DVec2, max: DVec2) -> bool {
        min.is_finite()
            && max.is_finite()
            && min.cmple(max).all()
            && min.cmpge(-self.usable).all()
            && max.cmple(self.usable).all()
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

    /// Visibility-graph route minimizing traversal through movable obstacles,
    /// then total distance. Fixed obstacles and perimeter remain hard constraints.
    pub fn plan_soft_route(
        self,
        start: DVec2,
        goal: DVec2,
        soft: &[Option<(DVec2, DVec2)>],
        waypoint_reserve: Length,
    ) -> Option<alloc::vec::Vec<DVec2>> {
        visibility::plan(self, start, goal, soft, waypoint_reserve)
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

/// Maximum projection of a vector rotated through any angle in [-yaw, yaw].
/// The maximum lies at the aligned direction or at one of the arc endpoints.
fn rotated_support(vector: DVec2, axis: DVec2, yaw: f64) -> f64 {
    let dot = vector.dot(axis);
    let cross = vector.perp_dot(axis).abs();
    if libm::atan2(cross, dot) <= yaw {
        vector.length()
    } else {
        dot * libm::cos(yaw) + cross * libm::sin(yaw)
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn braking_rotation_never_exceeds_the_turning_disk() {
        let chassis = Footprint::from_extents(
            crate::qty!(7.55 in),
            crate::qty!(7.55 in),
            crate::qty!(7.761804 in),
            crate::qty!(7.238196 in),
        )
        .unwrap();
        let field = RectangularClearance::new(
            chassis,
            crate::position_xy!(36.75 in, 54.5 in),
            crate::qty!(0.5 in),
        )
        .unwrap();
        let start = Pose::new(DVec2::new(0.48862, 0.01052), crate::qty!(-0.89179 rad));
        let (min, max) = field
            .braking_bounds(start, [0.0; 2], [0.0; 2], crate::qty!(180 deg))
            .unwrap();
        let radius = DVec2::splat(chassis.turn_radius());
        assert!((min - (start.position - radius)).length() < 1e-12);
        assert!((max - (start.position + radius)).length() < 1e-12);
        assert!(field.wall_contains_aabb(min, max));
    }

    #[test]
    fn braking_support_contains_sampled_combined_sweeps() {
        let chassis = Footprint::new(crate::qty!(15.1 in), crate::qty!(15 in)).unwrap();
        let field = RectangularClearance::new(
            chassis,
            crate::position_xy!(36.75 in, 54.5 in),
            crate::qty!(0.5 in),
        )
        .unwrap();
        let pose = Pose::new(DVec2::new(0.1, -0.2), crate::qty!(-51 deg));
        let yaw = crate::qty!(35 deg);
        let (min, max) = field
            .braking_bounds(pose, [-0.03, 0.2], [-0.08, 0.06], yaw)
            .unwrap();
        for i in -10..=10 {
            let heading = pose.heading + yaw * (f64::from(i) / 10.0);
            for f in [-0.03, 0.0, 0.2] {
                for l in [-0.08, 0.0, 0.06] {
                    let position = pose.position
                        + direction(heading) * f
                        + direction(heading - crate::qty!(90 deg)) * l;
                    let (a, b) = chassis.bounds(Pose::new(position, heading));
                    assert!((a + DVec2::splat(1e-12)).cmpge(min).all());
                    assert!((b - DVec2::splat(1e-12)).cmple(max).all());
                }
            }
        }
    }

    #[test]
    fn wall_stopping_envelope_is_checked_without_obstacles() {
        let chassis = Footprint::new(crate::qty!(15 in), crate::qty!(15 in)).unwrap();
        let field = RectangularClearance::new(
            chassis,
            crate::position_xy!(36.75 in, 54.5 in),
            crate::qty!(1 in),
        )
        .unwrap();
        let start = Pose::new(crate::position_xy!(0 in, 44 in), Angle::default());
        assert!(field.contains_pose(start));
        assert!(!field.wall_stop_clear(start, crate::position_xy!(0 in, 4 in), Angle::default(),));
        assert!(field.wall_stop_clear(start, crate::position_xy!(0 in, -4 in), Angle::default(),));
        assert!(!field.wall_stop_clear(start, DVec2::ZERO, crate::qty!(30 deg),));
    }

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
                crate::position_xy!(0 in, 36 in) * sign,
            ));
            let start = Pose::new(
                crate::position_xy!(20 in, 0 in) * sign + crate::position_xy!(0 in, -32 in),
                Angle::default(),
            );
            assert!(field.contains_translation(
                start,
                crate::position_xy!(20 in, 0 in) * sign + crate::position_xy!(0 in, 32 in),
            ));
        }
    }

    #[test]
    fn two_obstacle_gap_and_figure_eight_have_swept_clearance() {
        let chassis = Footprint::from_extents(
            crate::qty!(7.55 in),
            crate::qty!(7.55 in),
            crate::qty!(7.761804 in),
            crate::qty!(7.238196 in),
        )
        .unwrap();
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
        let east = crate::qty!(-90 deg);
        let left = Pose::new(crate::position_xy!(-20 in, 0 in), east);
        assert!(field.contains_translation(left, crate::position_xy!(20 in, 0 in)));
        let start = Pose::new(crate::position_xy!(12 in, 0 in), east);
        let mut pose = start;
        for sign in [1.0, -1.0] {
            for distance in [20.0, 24.0, 20.0, 24.0] {
                let radius = crate::qty!(8 in);
                let sweep = crate::qty!((90.0 * sign) deg);
                let (min, max) = chassis.forward_arc_bounds(pose, radius, sweep).unwrap();
                assert!(field.contains_aabb(min, max), "{pose:?}");
                pose = forward_arc_end(pose, radius, sweep);
                let end =
                    pose.position + direction(pose.heading) * crate::qty!((distance) in).value;
                assert!(field.contains_translation(pose, end), "{pose:?}");
                pose.position = end;
            }
            assert!(pose.position.distance(start.position) < 1e-9);
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
