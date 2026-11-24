//! Measurements shared by tracking sources and pose estimators.

use std::time::Duration;

use glam::DVec3;
use ozton_math::uom::si::f64::{Angle, Length};

use crate::{absolute::PoseObservation, inertial::Orientation, range::RangeObservation};

#[derive(Clone, Copy, Debug)]
pub enum DisplacementObservation {
    /// Cumulative travel of a tracking wheel along its mounted projection.
    ProjectedWheel(Length),
    /// Cumulative left and right drive wheel travel.
    Differential([Option<Length>; 2]),
}

#[derive(Clone, Copy, Debug)]
pub enum HeadingObservation {
    /// Clockwise heading from a sensor's zero, as reported by VEX IMUs.
    Compass(Angle),
    /// Counterclockwise heading from a sensor's zero.
    Counterclockwise(Angle),
}

/// Cumulative or instantaneous observations. Vector components are in SI units.
#[derive(Clone, Copy, Debug)]
pub enum Measurement {
    Displacement(DisplacementObservation),
    Heading(HeadingObservation),
    /// Angular rate in radians per second around chassis axes.
    Rotation(DVec3),
    /// Linear velocity in metres per second along chassis axes.
    Velocity(DVec3),
    /// Acceleration in metres per second squared along chassis axes.
    Acceleration(DVec3),
    Attitude(Orientation),
    Range(RangeObservation),
    AbsolutePose(PoseObservation),
}

/// A measurement tagged with its source index and timestamp on the tracking clock.
#[derive(Clone, Copy, Debug)]
pub struct Observation {
    pub source: usize,
    pub time: Duration,
    pub measurement: Measurement,
}

/// The sampling instant and output sink for one source. A source can emit any
/// combination of measurements and can leave the sink empty when unavailable.
pub struct MeasurementSink<'a> {
    pub time: Duration,
    pub range_occluded: bool,
    pub(crate) range_index: Option<usize>,
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
