//! Visibility graph over conservatively inflated axis-aligned obstacle boxes.
//! Dijkstra uses a lexicographic cost: soft overlap distance, then route length.
use alloc::vec::Vec;

use glam::DVec2;
use uom::si::f64::Length;

use super::RectangularClearance;
use crate::geometry::Pose;

type Bounds = (DVec2, DVec2);

#[derive(Clone, Copy)]
struct Cost {
    overlap: f64,
    length: f64,
}

impl Cost {
    const INFINITY: Self = Self {
        overlap: f64::INFINITY,
        length: f64::INFINITY,
    };

    fn compare(self, other: Self) -> core::cmp::Ordering {
        self.overlap
            .total_cmp(&other.overlap)
            .then_with(|| self.length.total_cmp(&other.length))
    }
}

/// Liang-Barsky/slab clipping: interval of the segment inside an AABB.
/// A corner touch counts as an intersection for hard obstacles.
fn clipped(a: DVec2, b: DVec2, bounds: Bounds) -> Option<(f64, f64)> {
    let (min, max) = bounds;
    let d = b - a;
    let mut enter: f64 = 0.0;
    let mut leave: f64 = 1.0;
    for axis in 0..2 {
        if d[axis].abs() <= f64::EPSILON {
            if a[axis] < min[axis] || a[axis] > max[axis] {
                return None;
            }
        } else {
            let t0 = (min[axis] - a[axis]) / d[axis];
            let t1 = (max[axis] - a[axis]) / d[axis];
            enter = enter.max(t0.min(t1));
            leave = leave.min(t0.max(t1));
            if enter > leave {
                return None;
            }
        }
    }
    Some((enter, leave))
}

pub(super) fn plan(
    field: RectangularClearance,
    start: DVec2,
    goal: DVec2,
    soft: &[Option<Bounds>],
    reserve: Length,
) -> Option<Vec<DVec2>> {
    if !start.is_finite() || !goal.is_finite() || !reserve.value.is_finite() || reserve.value <= 0.0
    {
        return None;
    }
    let radius = field.footprint.turn_radius();
    let inflate = |(min, max): Bounds, pad: f64| (min - DVec2::splat(pad), max + DVec2::splat(pad));
    let hard: Vec<_> = field
        .obstacles
        .iter()
        .flatten()
        .copied()
        .map(|bounds| inflate(bounds, radius))
        .collect();
    let mut soft_boxes = Vec::new();
    for &(min, max) in soft.iter().flatten() {
        if !min.is_finite() || !max.is_finite() || !min.cmplt(max).all() {
            return None;
        }
        soft_boxes.push(inflate((min, max), radius + field.margin));
    }
    let fits = |p| field.can_turn(Pose::new(p, Default::default()));
    if !fits(start) || !fits(goal) {
        return None;
    }

    let mut nodes = Vec::new();
    nodes.push(start);
    nodes.push(goal);
    for &bounds in hard.iter().chain(&soft_boxes) {
        let (a, b) = inflate(bounds, reserve.value);
        for point in [a, b, DVec2::new(a.x, b.y), DVec2::new(b.x, a.y)] {
            if fits(point) {
                nodes.push(point);
            }
        }
    }
    let n = nodes.len();
    let mut costs = alloc::vec![Cost::INFINITY; n];
    let mut previous: Vec<Option<usize>> = alloc::vec![None; n];
    let mut visited = alloc::vec![false; n];
    costs[0] = Cost {
        overlap: 0.0,
        length: 0.0,
    };
    for _ in 0..n {
        let u = (0..n)
            .filter(|&i| !visited[i] && costs[i].length.is_finite())
            .min_by(|&a, &b| costs[a].compare(costs[b]))?;
        if u == 1 {
            let mut result = alloc::vec![goal];
            let mut i = u;
            while let Some(parent) = previous[i] {
                if parent == 0 {
                    result.reverse();
                    return Some(result);
                }
                result.push(nodes[parent]);
                i = parent;
            }
            return None;
        }
        visited[u] = true;
        for v in 0..n {
            if visited[v]
                || hard
                    .iter()
                    .any(|&b| clipped(nodes[u], nodes[v], b).is_some())
            {
                continue;
            }
            let length = nodes[u].distance(nodes[v]);
            let overlap = soft_boxes
                .iter()
                .filter_map(|&b| clipped(nodes[u], nodes[v], b))
                .map(|(a, b)| length * (b - a))
                .sum::<f64>();
            let candidate = Cost {
                overlap: costs[u].overlap + overlap,
                length: costs[u].length + length,
            };
            if candidate.compare(costs[v]).is_lt() {
                costs[v] = candidate;
                previous[v] = Some(u);
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{path::clearance::Footprint, qty};

    #[test]
    fn avoids_movable_box_and_recovers_from_inside_it() {
        let field = RectangularClearance::new(
            Footprint::new(qty!(15 in), qty!(15 in)).unwrap(),
            DVec2::new(0.94, 1.38),
            qty!(1 in),
        )
        .unwrap();
        let obstacle = (DVec2::new(-0.05, -0.05), DVec2::new(0.05, 0.05));
        let soft = [Some(obstacle)];
        let start = DVec2::new(0.0, -0.8);
        let goal = DVec2::new(0.0, 0.8);
        let route = plan(field, start, goal, &soft, qty!(1 in)).unwrap();
        assert!(route.len() > 1);
        let inflated = (
            obstacle.0 - DVec2::splat(field.footprint.turn_radius() + field.margin),
            obstacle.1 + DVec2::splat(field.footprint.turn_radius() + field.margin),
        );
        let mut previous = start;
        for point in route {
            assert!(clipped(previous, point, inflated).is_none());
            previous = point;
        }
        // A moved landmark may overlap the start. This is a soft cost, not a
        // fatal wall/clearance condition.
        assert!(plan(field, DVec2::ZERO, goal, &soft, qty!(1 in)).is_some());
    }
}
