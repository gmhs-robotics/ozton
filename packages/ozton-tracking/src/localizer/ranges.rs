//! Range observation validation and single-use accounting.

use std::time::Duration;

use ozton_math::{
    geometry::{BodyDelta, Pose, wrap_heading},
    localization::range::{
        Constraint, Mount, RangeGate, RangeReading, Rejection, Surface, constrain_masked,
        fit_translation,
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
    /// Eligible map surfaces, e.g. walls reaching this sensor's mounting height.
    pub surface_mask: u32,
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
            surface_mask: u32::MAX,
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
    let reading = sensor.reading(sample, age);
    constrain_masked(
        pose,
        sensor.mount,
        reading,
        sensor.gate,
        map,
        sensor.surface,
        sensor.surface_mask,
    )
}

pub(super) struct ResetWindow {
    anchor: Option<(Duration, Pose)>,
    samples: [Option<RangeObservation>; MAX_RANGES],
    collecting: Option<Duration>,
    minimum_axes: u8,
}

impl Default for ResetWindow {
    fn default() -> Self {
        Self::new(1)
    }
}

pub(super) enum ResetResult {
    Waiting,
    Applied(Pose, u8),
    Rejected,
}

impl ResetWindow {
    pub(super) fn new(minimum_axes: u8) -> Self {
        Self {
            anchor: None,
            samples: [None; MAX_RANGES],
            collecting: None,
            minimum_axes,
        }
    }

    pub(super) fn require_axes(&mut self, minimum_axes: u8) {
        if minimum_axes > self.minimum_axes {
            *self = Self::new(minimum_axes);
        }
    }

    pub(super) fn restart(&mut self) {
        *self = Self::new(self.minimum_axes);
    }
    /// Distance sensors publish independently. Collect fresh, nearby samples while stopped.
    pub(super) fn update(
        &mut self,
        now: Duration,
        pose: Pose,
        map: &[Surface],
        sensors: &[RangeSensorConfig],
        reports: &[Option<RangeReport>; MAX_RANGES],
        maximum: f64,
        velocity: [f64; 3],
    ) -> ResetResult {
        if !velocity.iter().all(|v| v.is_finite())
            || velocity[0].hypot(velocity[1]) > 0.025
            || velocity[2].abs() > 0.06
        {
            self.restart();
            return ResetResult::Waiting;
        }
        if self.anchor.is_some_and(|(_, anchor)| {
            pose.position.distance(anchor.position) > 0.012
                || wrap_heading(pose.heading - anchor.heading).abs() > 0.02
        }) {
            self.restart();
        }
        let (started, _) = *self.anchor.get_or_insert((now, pose));
        // Require 120 ms of measured rest before collecting any packets.
        if now - started < Duration::from_millis(120) {
            return ResetResult::Waiting;
        }
        let collection_start = *self.collecting.get_or_insert(now);
        for (stored, report) in self.samples.iter_mut().zip(reports) {
            if let Some(report) = report {
                match report.constraint {
                    Ok(_) => {
                        *stored = report
                            .observation
                            .filter(|v| v.timestamp >= collection_start)
                    }
                    Err(Rejection::Duplicate)
                        if report.observation.is_some_and(|sample| !sample.occluded) => {}
                    Err(_) => *stored = None,
                }
            }
        }
        if now - collection_start < Duration::from_millis(45) {
            return ResetResult::Waiting;
        }
        let mut constraints = [(Constraint::default(), 0.0); MAX_RANGES];
        let mut count = 0;
        for constraint in sensors
            .iter()
            .zip(self.samples)
            .filter_map(|(&sensor, sample)| {
                let sample = sample?;
                let age = now.checked_sub(sample.timestamp)?.as_secs_f64();
                let c = constrain_sample(pose, map, sensor, sample, age).ok()?;
                Some((c, sensor.sigma * c.hit.incidence.abs()))
            })
        {
            constraints[count] = constraint;
            count += 1;
        }
        if count == 0 {
            return ResetResult::Waiting;
        }
        if let Some((change, axes)) = fit_translation(&constraints[..count]) {
            if axes < self.minimum_axes {
                return ResetResult::Waiting;
            }
            if change.length() <= maximum {
                return ResetResult::Applied(
                    Pose {
                        position: pose.position + change,
                        ..pose
                    },
                    axes,
                );
            }
        }
        ResetResult::Rejected
    }
}
