//! Target-aware departures in an empty rectangular field.

use super::*;
use crate::geometry::{bearing, wrap_heading};

/// Checked arc/straight departure to a place where the chassis can pivot toward the goal.
#[derive(Clone, Copy, Debug)]
pub struct PointDeparture {
    pub arc: Option<DepartureArc>,
    pub advance: Length,
}

impl RectangularClearance {
    /// Select a departure using total travel to the destination, including
    /// both pivots. Reverse approaches can use the shorter heading change.
    pub fn point_departure(
        self,
        start: Pose,
        target: Pose,
        half_track: Length,
        reserve: Length,
    ) -> Option<PointDeparture> {
        if !start.is_finite()
            || !target.is_finite()
            || !self.can_turn(target)
            || !half_track.value.is_finite()
            || half_track.value <= 0.0
        {
            return None;
        }
        let candidate = |arc: Option<DepartureArc>| {
            let end = arc.map_or(start, |arc| forward_arc_end(start, arc.radius, arc.sweep));
            let advance = self.forward_to_turn(end, reserve)?;
            let pivot_position = end.position + direction(end.heading) * advance.value;
            let displacement = target.position - pivot_position;
            let (_, turn_travel) = approach_heading(end.heading, target.heading, displacement);
            let curved = if let Some(arc) = arc {
                let (min, max) = self
                    .footprint
                    .forward_arc_bounds(start, arc.radius, arc.sweep)?;
                if !self.contains_bounds(min, max) {
                    return None;
                }
                (arc.radius.value + half_track.value) * arc.sweep.value.abs()
            } else {
                0.0
            };
            // Account for an additional slowdown and stop between legs.
            let straight_stage = if arc.is_some() && advance.value > reserve.value {
                0.06
            } else {
                0.0
            };
            let cost = curved
                + advance.value
                + displacement.length()
                + half_track.value * turn_travel
                + straight_stage;
            cost.is_finite()
                .then_some((PointDeparture { arc, advance }, cost))
        };

        core::iter::once(None)
            .chain(self.arcs(half_track).map(Some))
            .filter_map(candidate)
            .min_by(|(_, left), (_, right)| left.total_cmp(right))
            .map(|(departure, _)| departure)
    }
}

/// Select forward or reverse travel by the total heading change at both ends.
pub fn approach_heading(start: Angle, finish: Angle, displacement: DVec2) -> (bool, f64) {
    let forward = bearing(displacement);
    let reverse = forward + crate::angle!(core::f64::consts::PI, rad);
    let cost = |heading| {
        wrap_heading(heading - start).value.abs() + wrap_heading(finish - heading).value.abs()
    };
    let forward_cost = cost(forward);
    let reverse_cost = cost(reverse);
    (reverse_cost < forward_cost, forward_cost.min(reverse_cost))
}
