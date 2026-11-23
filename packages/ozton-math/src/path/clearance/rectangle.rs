//! Departures and turning clearance inside a centered rectangular perimeter.

use glam::DVec2;
use uom::si::{
    angle::degree,
    f64::{Angle, Length},
    length::meter,
};

use super::{Footprint, forward_arc_end};
use crate::geometry::{Pose, direction};

#[derive(Clone, Copy, Debug)]
pub struct RectangularClearance {
    footprint: Footprint,
    usable: DVec2,
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
            .then_some(Self { footprint, usable })
    }

    pub fn can_turn(self, pose: Pose) -> bool {
        pose.is_finite() && pose.position.abs().cmple(self.turn_area(0.0)).all()
    }

    fn turn_area(self, reserve: f64) -> DVec2 {
        self.usable - DVec2::splat(self.footprint.turn_radius() + reserve)
    }

    fn contains_bounds(self, min: DVec2, max: DVec2) -> bool {
        min.cmpge(-self.usable).all() && max.cmple(self.usable).all()
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
        (enter <= leave && self.contains_bounds(min + displacement, max + displacement))
            .then_some(Length::new::<meter>(enter))
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
        let first_radius = minimum_radius.value.max(0.05);
        let radius_step = self.usable.max_element() / 10.0;
        (0..=40)
            .map(|step| Length::new::<meter>(first_radius + radius_step * f64::from(step)))
            .flat_map(|radius| {
                (-18..=18)
                    .filter(|step| *step != 0)
                    .map(move |step| DepartureArc {
                        radius,
                        sweep: Angle::new::<degree>(5.0 * f64::from(step)),
                    })
            })
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
}

#[cfg(test)]
mod tests {
    use uom::si::length::inch;

    use super::*;

    #[test]
    fn corner_departure_needs_space_and_reaches_a_turning_area() {
        let inches = Length::new::<inch>;
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
