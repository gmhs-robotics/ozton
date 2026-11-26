//! Monotonic geometric progress along a path.

use glam::DVec2;
use uom::si::f64::Length;

use super::Path;
use crate::geometry::{circle_segment_fractions, direction};

#[derive(Debug)]
pub struct Progress<'path> {
    path: &'path Path,
    distance: f64,
    segment: usize,
}

impl<'path> Progress<'path> {
    pub fn new(path: &'path Path) -> Self {
        Self {
            path,
            distance: 0.0,
            segment: 0,
        }
    }

    pub fn path(&self) -> &'path Path {
        self.path
    }

    pub fn distance(&self) -> f64 {
        self.distance
    }

    /// Initialize another controller at a measured arc-length handoff.
    pub fn seek(&mut self, distance: f64) {
        self.distance = distance.clamp(0.0, self.path.end().distance);
        self.segment = self
            .path
            .samples()
            .partition_point(|s| s.distance <= self.distance)
            .saturating_sub(1);
    }

    /// Advances within a bounded local window to disambiguate crossing paths.
    pub fn update(&mut self, position: DVec2, advance_limit: Length) {
        let path = self.path;
        let maximum = self.distance + advance_limit.value.max(0.0);
        let start = path.samples[self.segment];
        let current = path
            .samples
            .get(self.segment + 1)
            .map_or(start.pose.position, |end| {
                let fraction = (self.distance - start.distance) / (end.distance - start.distance);
                start.pose.position.lerp(end.pose.position, fraction)
            });
        let mut best_error = position.distance_squared(current);
        let mut best_distance = self.distance;
        for pair in path.samples[self.segment..].windows(2) {
            let (a, b) = (pair[0], pair[1]);
            if a.distance > maximum {
                break;
            }
            let segment = b.pose.position - a.pose.position;
            let length = b.distance - a.distance;
            let lower = if self.distance <= a.distance {
                0.0
            } else {
                ((self.distance - a.distance) / length).min(1.0)
            };
            let upper = if maximum >= b.distance {
                1.0
            } else {
                ((maximum - a.distance) / length).clamp(lower, 1.0)
            };
            let projection = ((position - a.pose.position).dot(segment) / segment.length_squared())
                .clamp(lower, upper);
            let error = position.distance_squared(a.pose.position + segment * projection);
            if error < best_error {
                best_error = error;
                best_distance = a.distance + projection * length;
            }
        }
        self.distance = best_distance;
        while path
            .samples
            .get(self.segment + 1)
            .is_some_and(|point| point.distance <= self.distance)
        {
            self.segment += 1;
        }
    }

    /// First forward intersection with a circle about the robot.
    pub fn lookahead(&self, position: DVec2, radius: f64) -> DVec2 {
        let path = self.path;
        // Bound the search in metres, not samples: dense host-generated paths
        // must not shorten the geometric lookahead at speed.
        let anchor = path
            .at_distance(crate::qty!((self.distance) m))
            .pose
            .position;
        let maximum = self.distance + position.distance(anchor) + 2.0 * radius;
        for pair in path.samples[self.segment..].windows(2) {
            if pair[0].distance > maximum {
                break;
            }
            let (a, b) = (pair[0], pair[1]);
            let [_, exit] = circle_segment_fractions(
                position,
                crate::qty!((radius) m),
                a.pose.position,
                b.pose.position,
            );
            if let Some(exit) = exit
                && a.distance + (b.distance - a.distance) * exit >= self.distance
            {
                return a.pose.position.lerp(b.pose.position, exit);
            }
        }
        // Extend the final tangent for a full-radius carrot when the remaining
        // path lies inside the lookahead circle. Only guidance is extended;
        // progress, braking distance and the physical endpoint are unchanged.
        let end = path.end();
        let offset = end.pose.position - position;
        if end.distance <= maximum && offset.length_squared() < radius * radius {
            let tangent = direction(end.pose.heading);
            let along = offset.dot(tangent);
            let reach = libm::sqrt(along * along + radius * radius - offset.length_squared());
            return end.pose.position + tangent * (reach - along);
        }
        path.at_distance(crate::qty!((self.distance + radius) m))
            .pose
            .position
    }
}

#[cfg(all(test, not(target_os = "vexos")))]
mod tests {
    use alloc::vec::Vec;

    use super::*;
    use crate::{
        path::{Limits, Waypoint},
        qty,
    };

    #[test]
    fn final_tangent_preserves_lookahead_without_extending_progress() {
        let limits = Limits::from_units(
            qty!(0.33 m),
            qty!(1 m/s),
            qty!(1 m/s),
            qty!(1 m/s^2),
            qty!(1 m/s^2),
            qty!(1 m/s^2),
        );
        let path = Path::new(
            &[
                Waypoint::new(DVec2::ZERO, 1.0),
                Waypoint::new(DVec2::Y, 1.0),
            ],
            limits,
        )
        .unwrap();
        let mut progress = Progress::new(&path);
        progress.seek(0.95);
        for position in [DVec2::new(0.02, 0.95), DVec2::new(0.02, 1.02)] {
            let carrot = progress.lookahead(position, 0.27);
            assert!((carrot.distance(position) - 0.27).abs() < 1e-10);
            assert!(carrot.y > 1.0);
            assert_eq!(progress.distance(), 0.95);
        }
    }

    #[test]
    fn dense_path_lookahead_reaches_the_circle() {
        let limits = Limits::from_units(
            qty!(0.33 m),
            qty!(1 m/s),
            qty!(1 m/s),
            qty!(1 m/s^2),
            qty!(1 m/s^2),
            qty!(1 m/s^2),
        );
        let points: Vec<_> = (0..=100)
            .map(|i| Waypoint::new(DVec2::new(0.0, i as f64 * 0.002), 1.0))
            .chain((1..=500).map(|i| Waypoint::new(DVec2::new(i as f64 * 0.002, 0.2), 1.0)))
            .collect();
        let path = Path::new(&points, limits).unwrap();
        let progress = Progress::new(&path);
        let carrot = progress.lookahead(DVec2::ZERO, 0.6);
        assert!((carrot.length() - 0.6).abs() < 1e-6, "{carrot:?}");
    }
}
