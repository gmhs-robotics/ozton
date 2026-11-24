//! A* on a convex configuration-space visibility graph.
//! Lexicographic objective: minimize soft-obstacle overlap, then distance.

use alloc::vec::Vec;

use glam::DVec2;
use uom::si::f64::Length;

use super::{ConvexPolygon, RectangularClearance};
use crate::geometry::Pose;

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

    fn heuristic(self, distance: f64) -> Self {
        Self {
            length: self.length + distance,
            ..self
        }
    }
}

/// A* visibility-graph planner for a differential robot that can pivot.
/// Convex obstacles are offset by the circumscribed footprint radius.
#[derive(Clone, Copy, Debug)]
pub struct AStar {
    waypoint_reserve: Length,
}

impl Default for AStar {
    fn default() -> Self {
        Self::new()
    }
}

impl AStar {
    pub const fn new() -> Self {
        Self {
            waypoint_reserve: crate::qty!(1 in),
        }
    }

    #[must_use]
    pub const fn with_waypoint_reserve(mut self, reserve: Length) -> Self {
        self.waypoint_reserve = reserve;
        self
    }

    pub fn plan(
        self,
        field: &RectangularClearance,
        start: DVec2,
        goal: DVec2,
        soft: &[Option<ConvexPolygon>],
    ) -> Option<Vec<DVec2>> {
        if !start.is_finite()
            || !goal.is_finite()
            || !self.waypoint_reserve.value.is_finite()
            || self.waypoint_reserve.value <= 0.0
        {
            return None;
        }
        let pad = field.footprint.turn_radius() + field.margin;
        let fits = |p| field.can_turn(Pose::new(p, Default::default()));
        if !fits(start) || !fits(goal) {
            return None;
        }
        let hard = || field.obstacles.iter().flatten();
        let soft = || soft.iter().flatten();
        let mut nodes = Vec::new();
        nodes.push(start);
        nodes.push(goal);
        for polygon in hard().chain(soft()) {
            for index in 0..polygon.vertices().len() {
                if let Some(point) = polygon.offset_vertex(index, pad + self.waypoint_reserve.value)
                    && fits(point)
                {
                    nodes.push(point);
                }
            }
        }
        let n = nodes.len();
        let heuristics: Vec<_> = nodes.iter().map(|p| p.distance(goal)).collect();
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
                .min_by(|&a, &b| {
                    costs[a]
                        .heuristic(heuristics[a])
                        .compare(costs[b].heuristic(heuristics[b]))
                })?;
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
                if visited[v] || hard().any(|p| p.clip_segment(nodes[u], nodes[v], pad).is_some()) {
                    continue;
                }
                let length = nodes[u].distance(nodes[v]);
                let overlap = soft()
                    .filter_map(|p| p.clip_segment(nodes[u], nodes[v], pad))
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
        let soft = [ConvexPolygon::rectangle(obstacle.0, obstacle.1)];
        let start = DVec2::new(0.0, -0.8);
        let goal = DVec2::new(0.0, 0.8);
        let route = AStar::new()
            .with_waypoint_reserve(qty!(1 in))
            .plan(&field, start, goal, &soft)
            .unwrap();
        assert!(route.len() > 1);
        let inflated = (
            obstacle.0 - DVec2::splat(field.footprint.turn_radius() + field.margin),
            obstacle.1 + DVec2::splat(field.footprint.turn_radius() + field.margin),
        );
        let mut previous = start;
        for point in route {
            assert!(
                ConvexPolygon::rectangle(inflated.0, inflated.1)
                    .unwrap()
                    .clip_segment(previous, point, 0.0)
                    .is_none()
            );
            previous = point;
        }
        // A moved landmark may overlap the start. This is a soft cost, not a
        // fatal wall/clearance condition.
        assert!(
            AStar::new()
                .with_waypoint_reserve(qty!(1 in))
                .plan(&field, DVec2::ZERO, goal, &soft)
                .is_some()
        );
    }
}
