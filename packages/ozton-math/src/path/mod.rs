//! Prepared differential paths shared by importers and followers.

use alloc::borrow::Cow;
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
    /// Authored linear velocity ceiling before acceleration/braking parameterization.
    pub speed_cap: f64,
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
            let speed_cap = point.speed_fraction * limits.linear_speed.value;
            let mut speed = speed_cap.min(
                limits.wheel_speed.value / (1.0 + curvature.abs() * limits.track_width.value / 2.0),
            );
            if curvature != 0.0 {
                speed = speed.min(crate::numeric::sqrt(
                    limits.lateral_acceleration.value / curvature.abs(),
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
                speed_cap,
                time: 0.0,
            });
            incoming_length = outgoing_length;
        }
        parameterize(&mut samples, limits, &mut checkpoint).await?;
        Ok(Self {
            samples: Cow::Owned(samples),
            limits,
        })
    }

    /// Reparameterize tangent-connected paths as one continuous trajectory.
    /// Authored speed caps survive the original stop-at-end profiles.
    /// The caller supplies the common physical limits. No geometric replanning occurs.
    pub async fn join_cooperative<F>(
        paths: &[&Path],
        limits: Limits,
        mut checkpoint: F,
    ) -> Result<Self, PathError>
    where
        F: core::ops::AsyncFnMut() -> bool,
    {
        if !limits.is_valid() {
            return Err(PathError::InvalidLimits);
        }
        let mut samples: Vec<PathSample> = Vec::new();
        for path in paths {
            if (path.limits.track_width.value - limits.track_width.value).abs() > 1e-8 {
                return Err(PathError::InvalidLimits);
            }
            if let Some(previous) = samples.last_mut() {
                let start = path.start();
                if previous.pose.position.distance(start.pose.position) > 1e-6
                    || crate::geometry::wrap_heading(previous.pose.heading - start.pose.heading)
                        .value.abs() > 0.04
                {
                    return Err(PathError::InvalidPoint(samples.len() - 1));
                }
                previous.speed_cap = previous.speed_cap.min(start.speed_cap);
            }
            let offset = samples.last().map_or(0.0, |s| s.distance);
            let skip = usize::from(!samples.is_empty());
            for (i, source) in path.samples().iter().enumerate().skip(skip) {
                if i % 32 == 0 && !checkpoint().await {
                    return Err(PathError::Cancelled);
                }
                if samples.len() >= MAX_POINTS {
                    return Err(PathError::TooManyPoints);
                }
                let mut sample = *source;
                sample.distance += offset;
                sample.time = 0.0;
                samples.push(sample);
            }
        }
        if samples.len() < 3 {
            return Err(PathError::TooFewPoints);
        }
        // Recompute the tangent/curvature at joined samples, exactly as in
        // ordinary path generation; this also preserves curvature continuity.
        let last = samples.len() - 1;
        for i in 0..=last {
            if i % 32 == 0 && !checkpoint().await {
                return Err(PathError::Cancelled);
            }
            let before = samples[i.saturating_sub(1)].pose.position;
            let after = samples[(i + 1).min(last)].pose.position;
            let position = samples[i].pose.position;
            let incoming = position - before;
            let outgoing = after - position;
            let chord = after - before;
            let cross = incoming.perp_dot(outgoing);
            if i > 0 && i < last && incoming.dot(outgoing) < 0.0 && cross.abs() < 1e-9 {
                return Err(PathError::InvalidPoint(i));
            }
            let denominator = incoming.length() * outgoing.length() * chord.length();
            let curvature = if denominator > 0.0 { 2.0 * cross / denominator } else { 0.0 };
            let sample = &mut samples[i];
            sample.curvature = curvature;
            sample.pose.heading = crate::geometry::bearing(chord);
            sample.speed = sample.speed_cap.min(limits.linear_speed.value).min(
                limits.wheel_speed.value / (1.0 + curvature.abs() * limits.track_width.value / 2.0),
            );
            if curvature != 0.0 {
                sample.speed = sample.speed.min(crate::numeric::sqrt(
                    limits.lateral_acceleration.value / curvature.abs(),
                ));
            }
        }
        parameterize(&mut samples, limits, &mut checkpoint).await?;
        Ok(Self { samples: Cow::Owned(samples), limits })
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
                || !sample.speed_cap.is_finite()
                || sample.speed_cap < 0.0
                || sample.speed > sample.speed_cap + 1e-8
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
        speed_cap: a.speed_cap + (b.speed_cap - a.speed_cap) * fraction,
        distance,
        time,
    }
}

fn reachable(speed: f64, acceleration: f64, distance: f64) -> f64 {
    crate::numeric::sqrt(speed * speed + 2.0 * acceleration * distance)
}

/// Shared forward/backward velocity parameterization for authored and joined paths.
async fn parameterize<F>(
    samples: &mut [PathSample],
    limits: Limits,
    checkpoint: &mut F,
) -> Result<(), PathError>
where
    F: core::ops::AsyncFnMut() -> bool,
{
    let last = samples.len() - 1;
    samples[0].speed = 0.0;
    samples[last].speed = 0.0;
    for i in 1..=last {
        if i % 32 == 0 && !checkpoint().await {
            return Err(PathError::Cancelled);
        }
        let previous = samples[i - 1];
        let sample = &mut samples[i];
        sample.speed = sample.speed.min(reachable(
            previous.speed,
            limits.acceleration.value,
            sample.distance - previous.distance,
        ));
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
    parameterization::limit_wheels(samples, limits, checkpoint).await?;
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
    Ok(())
}

#[cfg(all(test, not(target_os = "vexos")))]
mod join_tests {
    use super::*;
    use crate::qty;

    #[test]
    fn joined_paths_match_one_continuous_profile_and_preserve_speed_caps() {
        let limits = Limits::from_units(qty!(0.33 m), qty!(1 m/s), qty!(1 m/s),
            qty!(1 m/s^2), qty!(1 m/s^2), qty!(1 m/s^2));
        let points: Vec<_> = (0..=20).map(|i| Waypoint::new(
            DVec2::new(0.0, i as f64 * 0.1), if i == 10 { 0.6 } else { 1.0 }
        )).collect();
        let full = Path::new(&points, limits).unwrap();
        let a = Path::new(&points[..=10], limits).unwrap();
        let b = Path::new(&points[10..], limits).unwrap();
        let joined = complete_ready(Path::join_cooperative(&[&a, &b], limits, async || true)).unwrap();
        assert!(joined.samples()[10].speed > 0.5);
        for (a, b) in joined.samples().iter().zip(full.samples()) {
            assert!((a.speed - b.speed).abs() < 1e-9);
            assert!((a.time - b.time).abs() < 1e-9);
        }
        assert_eq!(joined.start().speed, 0.0);
        assert_eq!(joined.end().speed, 0.0);
    }
}
