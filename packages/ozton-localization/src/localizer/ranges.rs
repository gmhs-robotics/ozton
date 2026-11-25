//! Range observation validation and single-use accounting.

use std::time::Duration;

use ozton_math::{
    geometry::Pose,
    localization::{
        observation::{RangeObservation, RangeRoles, RangeSensorConfig},
        range::{Constraint, Rejection, Surface, constrain_masked},
    },
    uom::si::{
        f64::{Length, Time},
        time::second,
    },
};

/// Compiled height-aware range roles. Masks never escape localization.
#[derive(Clone, Copy, Default)]
pub(super) struct RangeMasks {
    dsr: u8,
    ekf: u8,
    mcl: u8,
}

impl RangeMasks {
    pub(super) fn new(sensors: &[RangeSensorConfig]) -> Self {
        let mut masks = Self::default();
        for (index, sensor) in sensors.iter().enumerate() {
            if sensor.surface_mask == 0 {
                continue;
            }
            let bit = 1 << index;
            if sensor.roles.contains(RangeRoles::DSR) {
                masks.dsr |= bit;
            }
            if sensor.roles.contains(RangeRoles::EKF) {
                masks.ekf |= bit;
            }
            if sensor.roles.contains(RangeRoles::MCL) {
                masks.mcl |= bit;
            }
        }
        masks
    }

    pub(super) fn for_roles(self, roles: RangeRoles) -> u8 {
        let mut mask = 0;
        if roles.contains(RangeRoles::DSR) {
            mask |= self.dsr;
        }
        if roles.contains(RangeRoles::EKF) {
            mask |= self.ekf;
        }
        if roles.contains(RangeRoles::MCL) {
            mask |= self.mcl;
        }
        mask
    }

    pub(super) fn reset(self) -> u8 {
        self.dsr
    }
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
    pub now: Duration,
}

pub(super) fn inspect(
    tick: RangeTick,
    map: &[Surface],
    sensor: RangeSensorConfig,
    observation: Option<RangeObservation>,
    previous: &mut Option<Duration>,
    aligned: bool,
) -> RangeReport {
    let RangeTick { pose, now } = tick;
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
        if !aligned {
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

pub(super) fn constrain_sample(
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
        sensor.sample_gate(sample)?,
        map,
        sensor.surface,
        sensor.surface_mask,
    )
}
