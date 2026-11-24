//! Monotonic geometric progress along a path.

use glam::DVec2;
use uom::si::{f64::Length, length::meter};

use super::Path;
use crate::geometry::circle_segment_fractions;

#[derive(Debug)]
pub struct Progress<'path> {
    path: &'path Path,
    distance: f64,
}

impl<'path> Progress<'path> {
    pub fn new(path: &'path Path) -> Self {
        Self {
            path,
            distance: 0.0,
        }
    }

    pub fn path(&self) -> &'path Path {
        self.path
    }

    pub fn distance(&self) -> f64 {
        self.distance
    }

    /// Advances within a bounded local window to disambiguate crossing paths.
    pub fn update(&mut self, position: DVec2, advance_limit: Length) {
        let path = self.path;
        let maximum = self.distance + advance_limit.value.max(0.0);
        let mut best_error = position.distance_squared(
            path.at_distance(Length::new::<meter>(self.distance))
                .pose
                .position,
        );
        let mut best_distance = self.distance;
        for pair in path.samples[self.segment(path)..].windows(2).take(64) {
            let (a, b) = (pair[0], pair[1]);
            if a.distance > maximum {
                break;
            }
            let segment = b.pose.position - a.pose.position;
            let length = b.distance - a.distance;
            let lower = ((self.distance - a.distance) / length).clamp(0.0, 1.0);
            let upper = ((maximum - a.distance) / length).clamp(lower, 1.0);
            let projection = ((position - a.pose.position).dot(segment) / segment.length_squared())
                .clamp(lower, upper);
            let error = position.distance_squared(a.pose.position + segment * projection);
            if error < best_error {
                best_error = error;
                best_distance = a.distance + projection * length;
            }
        }
        self.distance = best_distance;
    }

    fn segment(&self, path: &Path) -> usize {
        path.samples
            .partition_point(|point| point.distance <= self.distance)
            .saturating_sub(1)
    }

    /// First forward intersection with a circle about the robot.
    pub fn lookahead(&self, position: DVec2, radius: f64) -> DVec2 {
        let path = self.path;
        for pair in path.samples[self.segment(path)..].windows(2).take(128) {
            let (a, b) = (pair[0], pair[1]);
            let [_, exit] = circle_segment_fractions(
                position,
                Length::new::<meter>(radius),
                a.pose.position,
                b.pose.position,
            );
            if let Some(exit) = exit
                && a.distance + (b.distance - a.distance) * exit >= self.distance
            {
                return a.pose.position.lerp(b.pose.position, exit);
            }
        }
        path.at_distance(Length::new::<meter>(self.distance + radius))
            .pose
            .position
    }
}
