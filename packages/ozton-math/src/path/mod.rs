//! Prepared differential paths shared by importers and followers.

use alloc::borrow::Cow;
#[cfg(not(target_os = "vexos"))]
use alloc::vec::Vec;

#[cfg(not(target_os = "vexos"))]
use glam::DVec2;
use uom::si::f64::{Acceleration, Length, Time, Velocity};

use crate::geometry::Pose;

pub mod braking;
#[cfg(not(target_os = "vexos"))]
mod chain;
#[cfg(not(target_os = "vexos"))]
pub use chain::PathChain;
pub mod clearance;
#[cfg(not(target_os = "vexos"))]
mod parameterization;
pub mod progress;

/// A +X-right/+Y-forward field coordinate in metres and a fraction of the configured maximum linear speed.
#[cfg(not(target_os = "vexos"))]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Waypoint {
    pub position: DVec2,
    pub speed_fraction: f64,
}

#[cfg(not(target_os = "vexos"))]
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
    /// Maximum absolute acceleration of either wheel.
    pub wheel_acceleration: Acceleration,
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
            wheel_acceleration: acceleration,
        }
    }

    #[must_use]
    pub const fn with_wheel_acceleration(mut self, value: Acceleration) -> Self {
        self.wheel_acceleration = value;
        self
    }

    /// Physical constraints in the same order used by generated path data.
    pub const fn as_array(self) -> [f64; 7] {
        [
            self.track_width.value,
            self.linear_speed.value,
            self.wheel_speed.value,
            self.acceleration.value,
            self.deceleration.value,
            self.lateral_acceleration.value,
            self.wheel_acceleration.value,
        ]
    }

    pub fn is_valid(self) -> bool {
        self.as_array()
            .iter()
            .all(|value| value.is_finite() && *value > 0.0)
    }

    /// Confirm a generated path uses the current calibration.
    pub fn matches(self, generated: [f64; 7]) -> bool {
        self.is_valid()
            && self
                .as_array()
                .into_iter()
                .zip(generated)
                .all(|(a, b)| b.is_finite() && (a - b).abs() <= 1e-10 * b.abs().max(1.0))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathError {
    InvalidLimits,
    InvalidPoint(usize),
    TooFewPoints,
    TooManyPoints,
    BlockedSegment(usize),
    Cancelled,
}

pub const MAX_POINTS: usize = 4096;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathSample {
    pub pose: Pose,
    pub curvature: f64,
    pub distance: f64,
    pub speed: f64,
    pub time: f64,
}

#[derive(Debug)]
pub struct Path {
    samples: Cow<'static, [PathSample]>,
    limits: Limits,
}

impl Path {
    /// Precomputes arc length, curvature and a forward/backward velocity pass.
    #[cfg(not(target_os = "vexos"))]
    pub fn new(points: &[Waypoint], limits: Limits) -> Result<Self, PathError> {
        complete_ready(Self::new_cooperative(points, limits, async || true))
    }

    /// Parameterize a path while letting a cooperative runtime service
    /// tracking between bounded batches of samples.
    #[cfg(not(target_os = "vexos"))]
    pub async fn new_cooperative<F>(
        points: &[Waypoint],
        limits: Limits,
        mut checkpoint: F,
    ) -> Result<Self, PathError>
    where
        F: core::ops::AsyncFnMut() -> bool,
    {
        if !limits.is_valid() {
            return Err(PathError::InvalidLimits);
        }
        if points.len() > MAX_POINTS {
            return Err(PathError::TooManyPoints);
        }
        for (index, point) in points.iter().enumerate() {
            if index % 32 == 0 && !checkpoint().await {
                return Err(PathError::Cancelled);
            }
            if !point.position.is_finite()
                || !(0.0..=1.0).contains(&point.speed_fraction)
                || (index > 0
                    && point.position.distance_squared(points[index - 1].position) < 1e-12)
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
            if index % 32 == 0 && !checkpoint().await {
                return Err(PathError::Cancelled);
            }
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
            if index > 0 && index < last && incoming.dot(outgoing) < 0.0 && cross.abs() < 1e-9 {
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
                speed = speed.min(crate::numeric::sqrt(
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
            if index % 32 == 0 && !checkpoint().await {
                return Err(PathError::Cancelled);
            }
            let next = samples[index + 1];
            let current = &mut samples[index];
            current.speed = current.speed.min(reachable(
                next.speed,
                limits.deceleration.value,
                next.distance - current.distance,
            ));
        }
        parameterization::limit_wheels(&mut samples, limits, &mut checkpoint).await?;
        for index in 1..=last {
            if index % 32 == 0 && !checkpoint().await {
                return Err(PathError::Cancelled);
            }
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
        Ok(Self {
            samples: Cow::Owned(samples),
            limits,
        })
    }

    /// Borrow a host-generated trajectory without allocating or parameterizing
    /// it on the controller. The producer must use the same physical limits
    /// and path algorithm; this checks the serialized sample invariants.
    pub fn from_precomputed(
        samples: &'static [PathSample],
        limits: Limits,
    ) -> Result<Self, PathError> {
        if !limits.is_valid() {
            return Err(PathError::InvalidLimits);
        }
        if samples.len() < 3 {
            return Err(PathError::TooFewPoints);
        }
        if samples.len() > MAX_POINTS {
            return Err(PathError::TooManyPoints);
        }
        for (i, sample) in samples.iter().enumerate() {
            if !sample.pose.is_finite()
                || !sample.curvature.is_finite()
                || !sample.distance.is_finite()
                || !sample.time.is_finite()
                || !sample.speed.is_finite()
                || sample.speed < 0.0
                || (i == 0 && (sample.distance != 0.0 || sample.time != 0.0))
                || ((i == 0 || i + 1 == samples.len()) && sample.speed != 0.0)
                || (i > 0
                    && (sample.distance <= samples[i - 1].distance
                        || sample.time <= samples[i - 1].time))
            {
                return Err(PathError::InvalidPoint(i));
            }
        }
        Ok(Self {
            samples: Cow::Borrowed(samples),
            limits,
        })
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
        let speed =
            crate::numeric::sqrt((a.speed * a.speed + 2.0 * acceleration * travel).max(0.0));
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

/// Synchronous algorithms supply an always-ready checkpoint. They share
/// the implementation with cooperative callers without requiring an executor.
#[cfg(not(target_os = "vexos"))]
pub(super) fn complete_ready<F: core::future::Future>(future: F) -> F::Output {
    let mut future = core::pin::pin!(future);
    let mut context = core::task::Context::from_waker(core::task::Waker::noop());
    match core::future::Future::poll(future.as_mut(), &mut context) {
        core::task::Poll::Ready(output) => output,
        core::task::Poll::Pending => unreachable!("ready checkpoint suspended"),
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

#[cfg(not(target_os = "vexos"))]
fn reachable(speed: f64, acceleration: f64, distance: f64) -> f64 {
    crate::numeric::sqrt(speed * speed + 2.0 * acceleration * distance)
}
