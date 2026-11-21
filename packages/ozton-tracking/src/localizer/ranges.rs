//! Range observation validation and single-use accounting.

use std::time::Duration;

use ozton_math::{
    geometry::{BodyDelta, Pose, wrap_heading},
    localization::range::{
        Constraint, Mount, RangeGate, RangeReading, Rejection, Surface, constrain_masked,
        fit_translation,
    },
    uom::{
        ConstZero,
        si::{
            angle::radian,
            f64::{Angle, Length, Time},
            time::second,
        },
    },
};

use crate::mount::Vec3;

pub const MAX_RANGES: usize = 8;

#[derive(Debug, Clone, Copy)]
pub struct RangeSensorConfig {
    pub mount: Mount,
    /// Beam height in the same chassis frame as the planar mount.
    pub mount_height: Length,
    pub gate: RangeGate,
    pub bias: Length,
    pub sigma: Length,
    pub surface: Option<usize>,
    /// Eligible map surfaces, e.g. walls reaching this sensor's mounting height.
    pub surface_mask: u32,
}

impl RangeSensorConfig {
    /// Default V5 range gates, no bias, and 25 mm observation noise.
    pub const fn new(mount: Mount) -> Self {
        Self {
            mount,
            mount_height: Length::ZERO,
            gate: RangeGate::DEFAULT,
            bias: Length::ZERO,
            sigma: Length {
                value: 0.025,
                ..Length::ZERO
            },
            surface: None,
            surface_mask: u32::MAX,
        }
    }

    /// Configure a horizontal beam from its chassis position and bearing.
    pub const fn at_mount(position: Vec3<Length>, bearing: Angle) -> Self {
        Self {
            mount_height: position.up,
            ..Self::new(Mount::new(position.right, position.forward, bearing))
        }
    }

    pub(super) fn valid(self, map: &[Surface]) -> bool {
        self.mount.offset.is_finite()
            && self.mount.heading.is_finite()
            && self.mount_height.value.is_finite()
            && self.gate.is_valid()
            && self.bias.value.is_finite()
            && self.sigma.value.is_finite()
            && self.sigma.value > 0.0
            && self.surface.is_none_or(|index| index < map.len())
    }

    /// A sensor-quality check for relative alignment. Pose association and
    /// innovation limits apply separately to localization corrections.
    pub fn relative_distance(
        self,
        sample: RangeObservation,
        now: Duration,
    ) -> Result<Length, Rejection> {
        let age = now.checked_sub(sample.timestamp).ok_or(Rejection::Stale)?;
        let reading = self.reading(sample, Time::new::<second>(age.as_secs_f64()));
        reading.validate(self.gate)?;
        Ok(reading.distance)
    }

    fn reading(self, sample: RangeObservation, age: Time) -> RangeReading {
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
    pub distance: Length,
    pub confidence: f64,
    pub occluded: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct RangeReport {
    pub observation: Option<RangeObservation>,
    pub calibrated_distance: Option<Length>,
    pub constraint: Result<Constraint, Rejection>,
}

#[derive(Clone, Copy)]
pub(super) struct RangeTick {
    pub pose: Pose,
    pub movement: BodyDelta,
    pub dt: Duration,
    pub now: Duration,
}

pub(super) fn inspect(
    tick: RangeTick,
    map: &[Surface],
    sensor: RangeSensorConfig,
    observation: Option<RangeObservation>,
    previous: &mut Option<Duration>,
) -> RangeReport {
    let RangeTick {
        pose,
        movement,
        dt,
        now,
    } = tick;
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
            + movement.heading.abs()
                * (sample.distance.value.abs() + sensor.mount.offset.length()))
            * age
            / dt.as_secs_f64();
        if motion_during_age > sensor.gate.max_residual.value * 0.25 {
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
    let reading = sensor.reading(sample, Time::new::<second>(age));
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

pub(super) struct ResetTick {
    pub time: Duration,
    pub pose: Pose,
    pub forward_velocity: f64,
    pub lateral_velocity: f64,
    pub angular_velocity: f64,
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
        tick: ResetTick,
        map: &[Surface],
        sensors: &[RangeSensorConfig],
        reports: &[Option<RangeReport>; MAX_RANGES],
        maximum: f64,
    ) -> ResetResult {
        let ResetTick {
            time: now,
            pose,
            forward_velocity,
            lateral_velocity,
            angular_velocity,
        } = tick;
        if ![forward_velocity, lateral_velocity, angular_velocity]
            .iter()
            .all(|v| v.is_finite())
            || forward_velocity.hypot(lateral_velocity) > 0.025
            || angular_velocity.abs() > 0.06
        {
            self.restart();
            return ResetResult::Waiting;
        }
        if self.anchor.is_some_and(|(_, anchor)| {
            pose.position.distance(anchor.position) > 0.012
                || wrap_heading(Angle::new::<radian>(pose.heading - anchor.heading)).abs() > 0.02
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
                Some((c, sensor.sigma.value * c.hit.incidence.abs()))
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
