//! Range observation validation and single-use accounting.

use std::time::Duration;

use ozton_math::{
    geometry::Pose,
    localization::{
        observation::{RangeObservation, RangeSensorConfig},
        range::{Constraint, Rejection, Surface, constrain_masked},
    },
    uom::si::{
        f64::{Length, Time},
        time::second,
    },
};

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
