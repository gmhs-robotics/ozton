//! Measurements shared by tracking sources and pose estimators.

use core::time::Duration;

use glam::DVec3;

use crate::uom::si::f64::{Angle, Length};

mod absolute;
mod capabilities;
mod orientation;
mod range;
mod snapshot;

pub use absolute::PoseObservation;
pub use capabilities::*;
pub use orientation::Orientation;
pub use range::{MAX_RANGES, RangeObservation, RangeSensorConfig};
pub use snapshot::Snapshot;

/// Observations captured on the same tick, in source order.
pub struct SensorSample<'a> {
    pub time: Duration,
    pub observations: &'a [Observation],
}

/// Fixed geometry keyed by the source indices used in samples.
#[derive(Clone, Copy, Default)]
pub struct ObservationLayout<'a> {
    pub wheels: &'a [(usize, DVec3)],
    pub ranges: &'a [(usize, RangeSensorConfig)],
}

/// A range-capable tracking system exposing its validated mount layout.
pub trait RangeLayoutSource: HasRange {
    fn observation_layout(&self) -> ObservationLayout<'_>;
}

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
