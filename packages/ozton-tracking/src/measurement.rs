//! The per-source measurement sink.

use std::time::Duration;

use crate::{Measurement, Observation};

/// The sampling instant and output sink for one source. A source can emit any
/// combination of measurements and can leave the sink empty when unavailable.
pub struct MeasurementSink<'a> {
    pub time: Duration,
    pub range_occluded: bool,
    /// This source's index among registered range sources, if it has range geometry.
    pub range_index: Option<usize>,
    pub(crate) source: usize,
    pub(crate) observations: &'a mut Vec<Observation>,
}

impl MeasurementSink<'_> {
    pub fn push(&mut self, measurement: Measurement) {
        let time = match measurement {
            Measurement::Range(range) => range.timestamp,
            Measurement::AbsolutePose(pose) => pose.time,
            _ => self.time,
        };
        self.observations.push(Observation {
            source: self.source,
            time,
            measurement,
        });
    }
}
