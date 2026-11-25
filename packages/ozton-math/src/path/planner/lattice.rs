//! Bounded A* over a position/heading lattice for a differential chassis.
//! Edges are exact straight translations or shortest in-place rotations.
use alloc::{
    collections::{BTreeMap, BinaryHeap},
    vec::Vec,
};
use core::{
    cmp::Ordering,
    f64::consts::{FRAC_PI_4, PI},
};

use super::RectangularClearance;
use crate::{
    geometry::{DVec2, Pose, bearing, wrap_heading},
    qty,
    uom::si::f64::{Angle, Length},
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LatticeStep {
    Turn(Angle),
    Translate { target: DVec2, reverse: bool },
}

#[derive(Debug, Clone, Copy)]
pub struct LatticePlanner {
    step: Length,
    node_limit: usize,
}

impl Default for LatticePlanner {
    fn default() -> Self {
        Self::new()
    }
}

impl LatticePlanner {
    pub const fn new() -> Self {
        Self {
            step: qty!(0.10 m),
            node_limit: 4096,
        }
    }

    #[must_use]
    pub const fn with_step(mut self, step: Length) -> Self {
        self.step = step;
        self
    }

    #[must_use]
    pub const fn with_node_limit(mut self, limit: usize) -> Self {
        self.node_limit = limit;
        self
    }

    /// Search using wheel travel as the cost and Euclidean distance as an
    /// admissible lower bound. Exact goal connectors avoid grid-snapping the
    /// final position or heading. The node cap bounds memory on the robot.
    pub fn plan(
        self,
        field: Option<&RectangularClearance>,
        start: Pose,
        goal: DVec2,
        heading: Option<Angle>,
        reverse: Option<bool>,
        track_width: Length,
    ) -> Option<Vec<LatticeStep>> {
        if !start.is_finite()
            || !goal.is_finite()
            || heading.is_some_and(|h| !h.value.is_finite())
            || !self.step.value.is_finite()
            || self.step.value <= 0.0
            || !track_width.value.is_finite()
            || track_width.value <= 0.0
            || self.node_limit == 0
            || field.is_some_and(|f| !f.contains_physical_pose(start))
        {
            return None;
        }
        let half_track = track_width.value * 0.5;
        let pose = |key: [i32; 3]| {
            Pose::new(
                start.transform_point(DVec2::new(key[0] as f64, key[1] as f64) * self.step.value),
                wrap_heading(start.heading + qty!((key[2] as f64 * FRAC_PI_4) rad)),
            )
        };
        let can_turn = |p, h| field.is_none_or(|f| f.contains_turn(p, h));
        let can_drive = |p, end| field.is_none_or(|f| f.contains_translation(p, end));
        let heuristic = |p: Pose| p.position.distance(goal);
        let root = Node {
            key: [0; 3],
            cost: 0.0,
            parent: None,
            step: None,
        };
        let mut nodes = alloc::vec![root];
        let mut known = BTreeMap::new();
        known.insert(root.key, 0);
        let mut open = BinaryHeap::new();
        open.push(Entry {
            cost: heuristic(start),
            index: 0,
        });
        let mut best: Option<(f64, usize, Vec<LatticeStep>)> = None;
        const DIRECTIONS: [[i32; 2]; 8] = [
            [0, 1],
            [-1, 1],
            [-1, 0],
            [-1, -1],
            [0, -1],
            [1, -1],
            [1, 0],
            [1, 1],
        ];
        while let Some(entry) = open.pop() {
            let node = nodes[entry.index];
            if known.get(&node.key) != Some(&entry.index) {
                continue;
            }
            if best
                .as_ref()
                .is_some_and(|(cost, _, _)| entry.cost >= *cost)
            {
                break;
            }
            let current = pose(node.key);
            // Connect to the exact goal by turning, translating, then turning.
            for backward in [false, true] {
                if reverse.is_some_and(|r| r != backward) {
                    continue;
                }
                let delta = goal - current.position;
                let distance = delta.length();
                let travel_heading = if distance > 1e-9 {
                    wrap_heading(bearing(delta) + qty!((if backward { PI } else { 0.0 }) rad))
                } else {
                    current.heading
                };
                let arrived = Pose::new(goal, travel_heading);
                if !can_turn(current, travel_heading)
                    || !can_drive(Pose::new(current.position, travel_heading), goal)
                    || heading.is_some_and(|h| !can_turn(arrived, h))
                {
                    continue;
                }
                let first = wrap_heading(travel_heading - current.heading).value.abs();
                let last = heading.map_or(0.0, |h| wrap_heading(h - travel_heading).value.abs());
                let cost = node.cost + distance + half_track * (first + last);
                if best.as_ref().is_some_and(|(c, _, _)| cost >= *c) {
                    continue;
                }
                let mut tail = Vec::new();
                if first > 1e-9 {
                    tail.push(LatticeStep::Turn(travel_heading));
                }
                if distance > 1e-9 {
                    tail.push(LatticeStep::Translate {
                        target: goal,
                        reverse: backward,
                    });
                }
                if last > 1e-9 {
                    tail.push(LatticeStep::Turn(heading.unwrap()));
                }
                best = Some((cost, entry.index, tail));
            }
            // No map requires no grid search.
            if field.is_none() {
                break;
            }
            let mut add = |key: [i32; 3], step, cost| {
                let cost = node.cost + cost;
                if nodes.len() >= self.node_limit
                    || known.get(&key).is_some_and(|&i| nodes[i].cost <= cost)
                {
                    return;
                }
                let priority = cost + heuristic(pose(key));
                if best.as_ref().is_some_and(|(c, _, _)| priority >= *c) {
                    return;
                }
                let index = nodes.len();
                nodes.push(Node {
                    key,
                    cost,
                    parent: Some(entry.index),
                    step: Some(step),
                });
                known.insert(key, index);
                open.push(Entry {
                    cost: priority,
                    index,
                });
            };
            for turn in [-1, 1] {
                let key = [node.key[0], node.key[1], (node.key[2] + turn).rem_euclid(8)];
                let h = pose(key).heading;
                if can_turn(current, h) {
                    add(key, LatticeStep::Turn(h), half_track * FRAC_PI_4);
                }
            }
            for backward in [false, true] {
                if reverse.is_some_and(|r| r != backward) {
                    continue;
                }
                let [dx, dy] = DIRECTIONS[node.key[2] as usize];
                let sign = if backward { -1 } else { 1 };
                let key = [
                    node.key[0] + dx * sign,
                    node.key[1] + dy * sign,
                    node.key[2],
                ];
                let target = pose(key).position;
                if can_drive(current, target) {
                    add(
                        key,
                        LatticeStep::Translate {
                            target,
                            reverse: backward,
                        },
                        current.position.distance(target),
                    );
                }
            }
        }
        let (_, mut index, tail) = best?;
        let mut steps = Vec::new();
        while let Some(parent) = nodes[index].parent {
            steps.push(nodes[index].step?);
            index = parent;
        }
        steps.reverse();
        steps.extend(tail);
        // Merge straight edges and consecutive rotations when the direct
        // swept rotation is clear. Neither operation changes the endpoint.
        let mut merged = Vec::new();
        let mut current = start;
        let mut turn_start = None;
        for step in steps {
            match step {
                LatticeStep::Turn(heading) => {
                    if let Some(origin) = turn_start
                        && let Some(LatticeStep::Turn(previous)) = merged.last_mut()
                        && can_turn(origin, heading)
                    {
                        *previous = heading;
                        current.heading = heading;
                        continue;
                    }
                    turn_start = Some(current);
                    current.heading = heading;
                }
                LatticeStep::Translate { target, reverse } => {
                    turn_start = None;
                    current.position = target;
                    if let Some(LatticeStep::Translate {
                        target: previous,
                        reverse: old,
                    }) = merged.last_mut()
                        && *old == reverse
                    {
                        *previous = target;
                        continue;
                    }
                }
            }
            merged.push(step);
        }
        Some(merged)
    }
}

#[derive(Clone, Copy)]
struct Node {
    key: [i32; 3],
    cost: f64,
    parent: Option<usize>,
    step: Option<LatticeStep>,
}

#[derive(Clone, Copy)]
struct Entry {
    cost: f64,
    index: usize,
}
impl PartialEq for Entry {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}
impl Eq for Entry {}
impl PartialOrd for Entry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Entry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .total_cmp(&self.cost)
            .then_with(|| other.index.cmp(&self.index))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::path::clearance::Footprint;

    #[test]
    fn straight_near_wall_does_not_require_a_full_turning_disk() {
        let field = RectangularClearance::new(
            Footprint::new(qty!(0.2 m), qty!(0.4 m)).unwrap(),
            DVec2::ONE,
            qty!(0 m),
        )
        .unwrap();
        let start = Pose::new(DVec2::new(0.88, -0.3), qty!(0 rad));
        let goal = DVec2::new(0.88, 0.3);
        assert!(!field.can_turn(start));
        let route = LatticePlanner::new()
            .plan(Some(&field), start, goal, None, None, qty!(0.3 m))
            .unwrap();
        assert_eq!(
            route,
            [LatticeStep::Translate {
                target: goal,
                reverse: false
            }]
        );
    }

    #[test]
    fn lattice_can_leave_a_wall_reserve_without_an_initial_pivot() {
        let field = RectangularClearance::new(
            Footprint::new(qty!(0.2 m), qty!(0.4 m)).unwrap(),
            DVec2::ONE,
            qty!(0.1 m),
        )
        .unwrap();
        let start = Pose::new(DVec2::new(0.0, -0.8), qty!(0 rad));
        let goal = DVec2::new(0.0, -0.4);
        let route = LatticePlanner::new()
            .plan(Some(&field), start, goal, None, None, qty!(0.3 m))
            .unwrap();
        assert_eq!(
            route,
            [LatticeStep::Translate {
                target: goal,
                reverse: false,
            }]
        );
    }

    #[test]
    fn fixed_obstacle_requires_a_collision_free_detour() {
        let field = RectangularClearance::new(
            Footprint::new(qty!(0.15 m), qty!(0.2 m)).unwrap(),
            DVec2::ONE,
            qty!(0.01 m),
        )
        .unwrap()
        .with_obstacle(DVec2::new(-0.2, -0.1), DVec2::new(0.2, 0.1))
        .unwrap();
        let start = Pose::new(DVec2::new(0.0, -0.6), qty!(0 rad));
        let goal = DVec2::new(0.0, 0.6);
        assert!(!field.contains_translation(start, goal));
        let route = LatticePlanner::new()
            .plan(
                Some(&field),
                start,
                goal,
                Some(qty!(0 rad)),
                None,
                qty!(0.3 m),
            )
            .unwrap();
        let mut pose = start;
        for step in route {
            match step {
                LatticeStep::Turn(heading) => {
                    assert!(field.contains_turn(pose, heading));
                    pose.heading = heading;
                }
                LatticeStep::Translate { target, .. } => {
                    assert!(field.contains_translation(pose, target));
                    pose.position = target;
                }
            }
        }
        assert_eq!(pose.position, goal);
        assert!(pose.heading.value.abs() < 1e-9);
    }
}
