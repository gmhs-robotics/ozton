//! Departures and turning clearance inside a centered rectangular perimeter.

use glam::DVec2;
use uom::si::{
    angle::degree,
    f64::{Angle, Length},
    length::meter,
};

use super::{Footprint, forward_arc_end};
use crate::geometry::{Pose, direction, rotation};

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
    pub fn new(footprint: Footprint, half_extents: [Length; 2], margin: Length) -> Option<Self> {
        let [right, forward] = half_extents;
        let usable = DVec2::new(right.value, forward.value) - DVec2::splat(margin.value);
        (usable.is_finite() && usable.min_element() > 0.0 && margin.value >= 0.0)
            .then_some(Self { footprint, usable })
    }

    pub fn can_turn(self, pose: Pose) -> bool {
        pose.is_finite() && pose.position.abs().cmple(self.turn_area(0.0)).all()
    }

    fn turn_area(self, reserve: f64) -> DVec2 {
        self.usable - DVec2::splat(self.footprint.half_extents.length() + reserve)
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

        let heading = Angle::new::<uom::si::angle::radian>(start.heading);
        let forward = direction(heading);
        let basis = rotation(heading).abs();
        let width = self.footprint.half_extents.x;
        let height = self.footprint.half_extents.y;
        let extent = basis * width + basis.perp().abs() * height;
        let straight_area = self.usable - extent;
        if !start.position.abs().cmple(straight_area).all() {
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
        (enter <= leave
            && (start.position + enter * forward)
                .abs()
                .cmple(straight_area)
                .all())
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
                (min.cmpge(-self.usable).all() && max.cmple(self.usable).all()).then_some((
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
        let chassis = Footprint::new(inches(15.1), inches(15.0)).unwrap();
        let field =
            RectangularClearance::new(chassis, [inches(19.0), inches(21.25)], inches(0.5)).unwrap();
        let gap = inches(0.25);
        let start = Pose::new(inches(-10.45), inches(-12.75), Angle::default());
        assert!(field.forward_to_turn(start, gap).is_none());
        let arc = field.departure_arc(start, inches(6.5), gap).unwrap();
        assert!(arc.sweep.value < 0.0);
        let end = forward_arc_end(start, arc.radius, arc.sweep);
        let advance = field.forward_to_turn(end, gap).unwrap();
        assert!(advance > inches(0.0));
        let turn = Pose {
            position: end.position
                + direction(Angle::new::<uom::si::angle::radian>(end.heading)) * advance.value,
            ..end
        };
        assert!(field.can_turn(turn));

        let flush = Pose::new(inches(-11.45), inches(-13.75), Angle::default());
        assert!(field.forward_to_turn(flush, gap).is_none());
        assert!(field.departure_arc(flush, inches(6.5), gap).is_none());
    }
}
