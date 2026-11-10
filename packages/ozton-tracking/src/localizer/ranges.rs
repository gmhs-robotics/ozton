//! Range observation validation and single-use accounting.

use std::time::Duration;

use ozton_math::{
    geometry::{BodyDelta, Pose, wrap_heading},
    localization::range::{
        Constraint, Mount, RangeGate, RangeReading, Rejection, Surface, constrain,
        reset_translation,
    },
};

pub const MAX_RANGES: usize = 8;

#[derive(Debug, Clone, Copy)]
pub struct RangeSensorConfig {
    pub mount: Mount,
    pub gate: RangeGate,
    pub bias: f64,
    pub sigma: f64,
    pub surface: Option<usize>,
}

impl RangeSensorConfig {
    /// Default V5 range gates, no bias, and 25 mm observation noise.
    pub const fn new(mount: Mount) -> Self {
        Self {
            mount,
            gate: RangeGate::DEFAULT,
            bias: 0.0,
            sigma: 0.025,
            surface: None,
        }
    }

    pub(super) fn valid(self, map: &[Surface]) -> bool {
        self.mount.offset.is_finite()
            && self.mount.heading.is_finite()
            && self.gate.is_valid()
            && self.bias.is_finite()
            && self.sigma.is_finite()
            && self.sigma > 0.0
            && self.surface.is_none_or(|index| index < map.len())
    }

    /// A sensor-quality check for relative alignment. Pose association and
    /// innovation limits apply separately to localization corrections.
    pub fn relative_distance(
        self,
        sample: RangeObservation,
        now: Duration,
    ) -> Result<f64, Rejection> {
        let age = now.checked_sub(sample.timestamp).ok_or(Rejection::Stale)?;
        let reading = self.reading(sample, age.as_secs_f64());
        reading.validate(self.gate)?;
        Ok(reading.distance)
    }

    fn reading(self, sample: RangeObservation, age: f64) -> RangeReading {
        RangeReading {
            distance: sample.distance - self.bias,
            confidence: sample.confidence,
            age,
            occluded: sample.occluded,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct RangeObservation {
    pub timestamp: Duration,
    pub distance: f64,
    pub confidence: f64,
    pub occluded: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct RangeReport {
    pub observation: Option<RangeObservation>,
    pub calibrated_distance: Option<f64>,
    pub constraint: Result<Constraint, Rejection>,
}

pub(super) fn inspect(
    pose: Pose,
    movement: BodyDelta,
    dt: Duration,
    now: Duration,
    map: &[Surface],
    sensor: RangeSensorConfig,
    observation: Option<RangeObservation>,
    previous: &mut Option<Duration>,
) -> RangeReport {
    let constraint = (|| {
        let sample = observation.ok_or(Rejection::Missing)?;
        if previous.is_some_and(|last| sample.timestamp <= last) {
            return Err(if sample.occluded {
                Rejection::Occluded
            } else {
                Rejection::Duplicate
            });
        }
        let age = now
            .checked_sub(sample.timestamp)
            .ok_or(Rejection::Stale)?
            .as_secs_f64();
        *previous = Some(sample.timestamp);
        let motion_during_age = (movement.translation.length()
            + movement.heading.abs() * (sample.distance.abs() + sensor.mount.offset.length()))
            * age
            / dt.as_secs_f64();
        if motion_during_age > sensor.gate.max_residual * 0.25 {
            return Err(Rejection::Stale);
        }
        constrain_sample(pose, map, sensor, sample, age)
    })();
    RangeReport {
        observation,
        calibrated_distance: observation.map(|sample| sample.distance - sensor.bias),
        constraint,
    }
}

fn constrain_sample(
    pose: Pose,
    map: &[Surface],
    sensor: RangeSensorConfig,
    sample: RangeObservation,
    age: f64,
) -> Result<Constraint, Rejection> {
    constrain(
        pose,
        sensor.mount,
        sensor.reading(sample, age),
        sensor.gate,
        map,
        sensor.surface,
    )
}

#[derive(Default)]
pub(super) struct ResetWindow {
    anchor: Option<(Duration, Pose)>,
    samples: [Option<RangeObservation>; MAX_RANGES],
}

pub(super) enum ResetResult {
    Waiting,
    Applied(Pose),
    Rejected,
}

impl ResetWindow {
    /// Distance sensors publish independently. Collect fresh, nearby samples while stopped.
    pub(super) fn update(
        &mut self,
        now: Duration,
        pose: Pose,
        map: &[Surface],
        sensors: &[RangeSensorConfig],
        reports: &[Option<RangeReport>; MAX_RANGES],
        maximum: f64,
    ) -> ResetResult {
        if self.anchor.is_some_and(|(_, anchor)| {
            pose.position.distance(anchor.position) > 0.012
                || wrap_heading(pose.heading - anchor.heading).abs() > 0.02
        }) {
            *self = Self::default();
        }
        let (started, _) = *self.anchor.get_or_insert((now, pose));
        for (stored, report) in self.samples.iter_mut().zip(reports) {
            if let Some(report) = report {
                match report.constraint {
                    Ok(_) => *stored = report.observation,
                    Err(Rejection::Duplicate)
                        if report.observation.is_some_and(|sample| !sample.occluded) => {}
                    Err(_) => *stored = None,
                }
            }
        }
        let mut constraints = sensors
            .iter()
            .zip(self.samples)
            .filter_map(|(&sensor, sample)| {
                let sample = sample?;
                let age = now.checked_sub(sample.timestamp)?.as_secs_f64();
                constrain_sample(pose, map, sensor, sample, age).ok()
            });
        let first = constraints.next();
        let second = first.and_then(|first| {
            constraints
                .find(|candidate| first.hit.normal.perp_dot(candidate.hit.normal).abs() >= 0.5)
        });
        if second.is_none() && sensors.len() > 1 && now - started < Duration::from_millis(45) {
            return ResetResult::Waiting;
        }
        if let Some(first) = first {
            let change = reset_translation(first, second);
            if change.length() <= maximum {
                return ResetResult::Applied(Pose {
                    position: pose.position + change,
                    ..pose
                });
            }
        }
        ResetResult::Rejected
    }
}
