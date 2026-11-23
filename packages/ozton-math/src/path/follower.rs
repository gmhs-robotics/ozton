//! Pure pursuit and RAMSETE tracking for the same prepared path.

use core::f64::consts::PI;

use glam::DVec2;
use uom::si::{
    angle::radian,
    angular_velocity::radian_per_second,
    f64::{Angle, AngularVelocity, Length, Time, Velocity},
    length::meter,
    time::second,
    velocity::meter_per_second,
};

use super::Path;
use crate::{
    desaturate,
    geometry::{Pose, circle_segment_fractions, pursuit_curvature, wheel_speeds, wrap_heading},
};

#[derive(Debug, Clone, Copy)]
pub enum Follower {
    PurePursuit { lookahead: Length },
    Ramsete { b: f64, zeta: f64 },
}

impl Follower {
    pub fn is_valid(self) -> bool {
        match self {
            Self::PurePursuit { lookahead } => lookahead.value.is_finite() && lookahead.value > 0.0,
            Self::Ramsete { b, zeta } => {
                b.is_finite() && b > 0.0 && (0.0..=1.0).contains(&zeta) && zeta > 0.0
            }
        }
    }
}

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

    fn lookahead(&self, position: DVec2, radius: f64) -> DVec2 {
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

/// Timing and pose for one follower tick.
#[derive(Clone, Copy, Debug)]
pub struct FollowerSample {
    pub pose: Pose,
    pub elapsed: Time,
    pub period: Time,
}

/// Wheel speeds in meters per second. The caller owns settling and stopping.
pub fn command(
    follower: Follower,
    progress: &Progress<'_>,
    sample: FollowerSample,
    reverse: bool,
    position_tolerance: Length,
) -> [f64; 2] {
    let path = progress.path;
    let pose = sample.pose;
    let (elapsed, dt, position_tolerance) = (
        sample.elapsed.value,
        sample.period.value,
        position_tolerance.value,
    );
    let sign = if reverse { -1.0 } else { 1.0 };
    let facing = Angle::new::<radian>(if reverse { PI } else { 0.0 });
    let end = path.end();
    let limits = path.limits();
    let (linear, angular) = match follower {
        Follower::PurePursuit { lookahead } => {
            let virtual_pose = Pose {
                heading: pose.heading + facing,
                ..pose
            };
            let target = virtual_pose.to_local(progress.lookahead(pose.position, lookahead.value));
            let reference = path.at_distance(Length::new::<meter>(progress.distance));
            let speed = path.at_time(Time::new::<second>(reference.time + dt)).speed;
            if target.y < 0.0 {
                (0.0, 2.0 * crate::geometry::bearing(target).value)
            } else {
                (sign * speed, speed * pursuit_curvature(target))
            }
        }
        Follower::Ramsete { b, zeta } => {
            let reference = path.at_time(Time::new::<second>(elapsed + dt));
            let error = pose.to_local(reference.pose.position);
            let heading_error = wrap_heading(reference.pose.heading + facing - pose.heading).value;
            let velocity = sign * reference.speed;
            let omega = reference.speed * reference.curvature;
            let gain = 2.0 * zeta * libm::sqrt(omega * omega + b * velocity * velocity);
            let sinc = if heading_error.abs() < 1e-5 {
                1.0
            } else {
                libm::sin(heading_error) / heading_error
            };
            (
                velocity * libm::cos(heading_error) + gain * error.y,
                omega + gain * heading_error + b * velocity * sinc * -error.x,
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
        let error = pose.to_local(end.pose.position);
        let heading_error = wrap_heading(end.pose.heading + facing - pose.heading).value;
        let distance = error.length();
        if distance > position_tolerance {
            let travel_heading = crate::geometry::bearing(error);
            let backward = travel_heading.value.abs() > PI / 2.0;
            let bearing = wrap_heading(
                travel_heading + Angle::new::<radian>(if backward { PI } else { 0.0 }),
            )
            .value;
            let terminal_speed = (2.0 * distance).min(limits.linear_speed.value * 0.4);
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
        wheel_speeds(
            Velocity::new::<meter_per_second>(linear),
            AngularVelocity::new::<radian_per_second>(angular),
            limits.track_width,
        )
        .map(|speed| speed.value),
        limits.wheel_speed.value,
    )
}
