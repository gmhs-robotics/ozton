//! Pure pursuit and RAMSETE tracking for the same prepared path.

use core::f64::consts::PI;

use glam::DVec2;

use super::Path;
use crate::{
    desaturate,
    geometry::{Pose, circle_segment_fractions, pursuit_curvature, wheel_speeds, wrap_heading},
};

#[derive(Debug, Clone, Copy)]
pub enum Follower {
    PurePursuit { lookahead: f64 },
    Ramsete { b: f64, zeta: f64 },
}

impl Follower {
    pub fn is_valid(self) -> bool {
        match self {
            Self::PurePursuit { lookahead } => lookahead.is_finite() && lookahead > 0.0,
            Self::Ramsete { b, zeta } => {
                b.is_finite() && b > 0.0 && (0.0..=1.0).contains(&zeta) && zeta > 0.0
            }
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Progress {
    distance: f64,
    segment: usize,
}

impl Progress {
    pub fn distance(&self) -> f64 {
        self.distance
    }

    /// Advances within a bounded local window to disambiguate crossing paths.
    pub fn update(&mut self, path: &Path, position: DVec2, advance_limit: f64) {
        let maximum = self.distance + advance_limit.max(0.0);
        let mut best_error = position.distance_squared(path.at_distance(self.distance).position);
        let mut best_distance = self.distance;
        for pair in path.samples[self.segment..].windows(2).take(64) {
            let (a, b) = (pair[0], pair[1]);
            if a.distance > maximum {
                break;
            }
            let segment = b.position - a.position;
            let length = b.distance - a.distance;
            let lower = ((self.distance - a.distance) / length).clamp(0.0, 1.0);
            let upper = ((maximum - a.distance) / length).clamp(lower, 1.0);
            let projection = ((position - a.position).dot(segment) / segment.length_squared())
                .clamp(lower, upper);
            let error = position.distance_squared(a.position + segment * projection);
            if error < best_error {
                best_error = error;
                best_distance = a.distance + projection * length;
            }
        }
        self.distance = best_distance;
        self.segment = path
            .samples
            .partition_point(|point| point.distance <= self.distance)
            .saturating_sub(1);
    }

    fn lookahead(&self, path: &Path, position: DVec2, radius: f64) -> DVec2 {
        for pair in path.samples[self.segment..].windows(2).take(128) {
            let (a, b) = (pair[0], pair[1]);
            let [_, exit] = circle_segment_fractions(position, radius, a.position, b.position);
            if let Some(exit) = exit
                && a.distance + (b.distance - a.distance) * exit >= self.distance
            {
                return a.position.lerp(b.position, exit);
            }
        }
        path.at_distance(self.distance + radius).position
    }
}

/// Wheel speeds in meters per second. The caller owns settling and stopping.
pub fn command(
    follower: Follower,
    path: &Path,
    progress: Progress,
    pose: Pose,
    elapsed: f64,
    dt: f64,
    reverse: bool,
    position_tolerance: f64,
) -> [f64; 2] {
    let sign = if reverse { -1.0 } else { 1.0 };
    let facing = if reverse { PI } else { 0.0 };
    let end = path.end();
    let limits = path.limits();
    let (linear, angular) = match follower {
        Follower::PurePursuit { lookahead } => {
            let virtual_pose = Pose {
                heading: pose.heading + facing,
                ..pose
            };
            let target = virtual_pose.to_local(progress.lookahead(path, pose.position, lookahead));
            let reference = path.at_distance(progress.distance);
            let speed = path.at_time(reference.time + dt).speed;
            if target.x < 0.0 {
                (0.0, 2.0 * libm::atan2(target.y, target.x))
            } else {
                (sign * speed, speed * pursuit_curvature(target))
            }
        }
        Follower::Ramsete { b, zeta } => {
            let reference = path.at_time(elapsed + dt);
            let error = pose.to_local(reference.position);
            let heading_error = wrap_heading(reference.heading + facing - pose.heading);
            let velocity = sign * reference.speed;
            let omega = reference.speed * reference.curvature;
            let gain = 2.0 * zeta * libm::sqrt(omega * omega + b * velocity * velocity);
            let sinc = if heading_error.abs() < 1e-5 {
                1.0
            } else {
                libm::sin(heading_error) / heading_error
            };
            (
                velocity * libm::cos(heading_error) + gain * error.x,
                omega + gain * heading_error + b * velocity * sinc * error.y,
            )
        }
    };
    let terminal = match follower {
        Follower::PurePursuit { .. } => {
            progress.distance >= end.distance - position_tolerance.max(0.025)
        }
        Follower::Ramsete { .. } => elapsed + dt >= end.time,
    };
    let (linear, angular) = if terminal {
        let error = pose.to_local(end.position);
        let heading_error = wrap_heading(end.heading + facing - pose.heading);
        let distance = error.length();
        if distance > position_tolerance {
            let travel_heading = libm::atan2(error.y, error.x);
            let backward = travel_heading.abs() > PI / 2.0;
            let bearing = wrap_heading(travel_heading + if backward { PI } else { 0.0 });
            let terminal_speed = (2.0 * distance).min(limits.linear_speed * 0.4);
            (
                terminal_speed * libm::cos(bearing) * if backward { -1.0 } else { 1.0 },
                3.0 * bearing,
            )
        } else {
            (0.0, 3.0 * heading_error)
        }
    } else {
        (linear, angular)
    };
    desaturate(
        wheel_speeds(linear, angular, limits.track_width),
        limits.wheel_speed,
    )
}
