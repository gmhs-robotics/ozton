//! Prepared differential paths shared by importers and followers.

use alloc::vec::Vec;

use glam::DVec2;
use uom::si::f64::{Acceleration, Length, Time, Velocity};

use crate::geometry::Pose;

pub mod clearance;
pub mod planner;
pub mod progress;

/// A +X-right/+Y-forward field coordinate in metres and a fraction of the configured maximum linear speed.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Waypoint {
    pub position: DVec2,
    pub speed_fraction: f64,
}

impl Waypoint {
    /// Creates a waypoint from right/forward field lengths.
    pub const fn new(position: DVec2, speed_fraction: f64) -> Self {
        Self {
            position,
            speed_fraction,
        }
    }
}

/// Physical constraints in SI units.
#[derive(Debug, Clone, Copy)]
pub struct Limits {
    pub track_width: Length,
    pub linear_speed: Velocity,
    pub wheel_speed: Velocity,
    pub acceleration: Acceleration,
    pub deceleration: Acceleration,
    pub lateral_acceleration: Acceleration,
}

impl Limits {
    /// Creates path limits from physical quantities in any supported units.
    pub const fn from_units(
        track_width: Length,
        linear_speed: Velocity,
        wheel_speed: Velocity,
        acceleration: Acceleration,
        deceleration: Acceleration,
        lateral_acceleration: Acceleration,
    ) -> Self {
        Self {
            track_width,
            linear_speed,
            wheel_speed,
            acceleration,
            deceleration,
            lateral_acceleration,
        }
    }

    pub fn is_valid(self) -> bool {
        [
            self.track_width.value,
            self.linear_speed.value,
            self.wheel_speed.value,
            self.acceleration.value,
            self.deceleration.value,
            self.lateral_acceleration.value,
        ]
        .iter()
        .all(|value| value.is_finite() && *value > 0.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathError {
    InvalidLimits,
    InvalidPoint(usize),
    TooFewPoints,
    TooManyPoints,
    BlockedSegment(usize),
}

pub const MAX_POINTS: usize = 4096;

#[derive(Debug, Clone, Copy)]
pub struct PathSample {
    pub pose: Pose,
    pub curvature: f64,
    pub distance: f64,
    pub speed: f64,
    pub time: f64,
}

#[derive(Debug)]
pub struct Path {
    samples: Vec<PathSample>,
    limits: Limits,
}

impl Path {
    /// Precomputes arc length, curvature and a forward/backward velocity pass.
    pub fn new(points: &[Waypoint], limits: Limits) -> Result<Self, PathError> {
        if !limits.is_valid() {
            return Err(PathError::InvalidLimits);
        }
        if points.len() > MAX_POINTS {
            return Err(PathError::TooManyPoints);
        }
        for (index, point) in points.iter().enumerate() {
            if !point.position.is_finite()
                || !(0.0..=1.0).contains(&point.speed_fraction)
                || (index > 0 && point.position.distance_squared(points[index - 1].position) < 1e-12)
            {
                return Err(PathError::InvalidPoint(index));
            }
        }
        let midpoint;
        let points = match points {
            [start, end] => {
                midpoint = [
                    *start,
                    Waypoint {
                        position: start.position.lerp(end.position, 0.5),
                        speed_fraction: start.speed_fraction.max(end.speed_fraction),
                    },
                    *end,
                ];
                &midpoint
            }
            [_, _, _, ..] => points,
            _ => return Err(PathError::TooFewPoints),
        };
        let mut samples: Vec<PathSample> = Vec::with_capacity(points.len());
        let last = points.len() - 1;
        let mut incoming_length = 0.0;
        for (index, point) in points.iter().enumerate() {
            let before = points[index.saturating_sub(1)].position;
            let after = points[(index + 1).min(last)].position;
            let incoming = point.position - before;
            let outgoing = after - point.position;
            let chord = after - before;
            let outgoing_length = outgoing.length();
            let cross = incoming.perp_dot(outgoing);
            let denominator = if index > 0 && index < last {
                incoming_length * outgoing_length * chord.length()
            } else {
                0.0
            };
            if index > 0
                && index < last
                && incoming.dot(outgoing) < 0.0
                && cross.abs() < 1e-9
            {
                return Err(PathError::InvalidPoint(index));
            }
            let curvature = if denominator > 0.0 {
                2.0 * cross / denominator
            } else {
                0.0
            };
            let distance = samples
                .last()
                .map_or(0.0, |sample| sample.distance + incoming_length);
            let mut speed = (point.speed_fraction * limits.linear_speed.value).min(
                limits.wheel_speed.value / (1.0 + curvature.abs() * limits.track_width.value / 2.0),
            );
            if curvature != 0.0 {
                speed = speed.min(libm::sqrt(
                    limits.lateral_acceleration.value / curvature.abs(),
                ));
            }
            if index == 0 || index == last {
                speed = 0.0;
            }
            if let Some(previous) = samples.last() {
                speed = speed.min(reachable(
                    previous.speed,
                    limits.acceleration.value,
                    incoming_length,
                ));
            }
            if !distance.is_finite() || !curvature.is_finite() {
                return Err(PathError::InvalidPoint(index));
            }
            samples.push(PathSample {
                pose: Pose::new(point.position, crate::geometry::bearing(chord)),
                curvature,
                distance,
                speed,
                time: 0.0,
            });
            incoming_length = outgoing_length;
        }
        for index in (0..last).rev() {
            let next = samples[index + 1];
            let current = &mut samples[index];
            current.speed = current.speed.min(reachable(
                next.speed,
                limits.deceleration.value,
                next.distance - current.distance,
            ));
        }
        for index in 1..=last {
            let previous = samples[index - 1];
            let current = &mut samples[index];
            let speed_sum = current.speed + previous.speed;
            if speed_sum <= 0.0 {
                return Err(PathError::BlockedSegment(index));
            }
            current.time = previous.time + 2.0 * (current.distance - previous.distance) / speed_sum;
            if !current.time.is_finite() {
                return Err(PathError::InvalidPoint(index));
            }
        }
        Ok(Self { samples, limits })
    }

    /// Samples of the prepared path in increasing arc length.
    pub fn samples(&self) -> &[PathSample] {
        &self.samples
    }

    pub fn limits(&self) -> Limits {
        self.limits
    }
    pub fn start(&self) -> PathSample {
        self.samples[0]
    }
    pub fn end(&self) -> PathSample {
        self.samples[self.samples.len() - 1]
    }
    pub fn at_distance(&self, distance: Length) -> PathSample {
        let distance = distance.value;
        let Some((a, b)) = self.bracket(distance, |sample| sample.distance) else {
            return if distance > 0.0 {
                self.end()
            } else {
                self.start()
            };
        };
        let travel = distance - a.distance;
        let acceleration = (b.speed - a.speed) / (b.time - a.time);
        let speed = libm::sqrt((a.speed * a.speed + 2.0 * acceleration * travel).max(0.0));
        let time = a.time + 2.0 * travel / (a.speed + speed);
        blend(a, b, distance, time, speed)
    }
    pub fn at_time(&self, time: Time) -> PathSample {
        let time = time.value;
        let Some((a, b)) = self.bracket(time, |sample| sample.time) else {
            return if time > 0.0 { self.end() } else { self.start() };
        };
        let elapsed = time - a.time;
        let speed = a.speed + (b.speed - a.speed) * elapsed / (b.time - a.time);
        let distance = (a.distance + (a.speed + speed) * elapsed / 2.0).min(b.distance);
        blend(a, b, distance, time, speed)
    }

    fn bracket(
        &self,
        query: f64,
        key: impl Fn(PathSample) -> f64,
    ) -> Option<(PathSample, PathSample)> {
        let upper = self.samples.partition_point(|sample| key(*sample) < query);
        (0 < upper && upper < self.samples.len())
            .then(|| (self.samples[upper - 1], self.samples[upper]))
    }
}

fn blend(a: PathSample, b: PathSample, distance: f64, time: f64, speed: f64) -> PathSample {
    let fraction = (distance - a.distance) / (b.distance - a.distance);
    PathSample {
        pose: a.pose.interpolate(b.pose, fraction),
        curvature: a.curvature + (b.curvature - a.curvature) * fraction,
        speed,
        distance,
        time,
    }
}

fn reachable(speed: f64, acceleration: f64, distance: f64) -> f64 {
    libm::sqrt(speed * speed + 2.0 * acceleration * distance)
}
