//! Timestamped device snapshots and travel source selection.

use std::time::Duration;

use ozton_math::{
    geometry::{BodyDelta, Pose, wrap_heading},
    localization::odometry::body_delta,
};

/// A GPS fix and its estimated position error, in meters.
#[derive(Debug, Clone, Copy)]
pub struct GpsFix {
    pub pose: Pose,
    pub position_error: f64,
    pub timestamp: Duration,
}

/// One tick's cumulative encoder positions in meters and heading in radians.
/// Unavailable or uncalibrated devices supply None.
#[derive(Debug, Clone, Copy, Default)]
pub struct Snapshot {
    pub time: Duration,
    pub drive: [Option<f64>; 2],
    pub forward: Option<f64>,
    pub lateral: Option<f64>,
    pub imu: Option<f64>,
    pub gps: Option<GpsFix>,
}

#[derive(Debug, Clone, Copy)]
pub struct OdometryConfig {
    pub track_width: f64,
    pub forward_offset: f64,
    pub lateral_offset: f64,
    /// Require lateral feedback, e.g. on a holonomic drivetrain.
    pub require_lateral: bool,
    pub max_linear_speed: f64,
    pub max_angular_speed: f64,
    pub max_gap: Duration,
}

impl OdometryConfig {
    pub fn is_valid(self) -> bool {
        [
            self.track_width,
            self.max_linear_speed,
            self.max_angular_speed,
        ]
        .iter()
        .all(|value| value.is_finite() && *value > 0.0)
            && self.forward_offset.is_finite()
            && self.lateral_offset.is_finite()
            && !self.max_gap.is_zero()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    Initializing,
    Time,
    Forward,
    Heading,
    Lateral,
    InvalidEstimate,
}

pub(crate) fn delta(
    previous: Snapshot,
    next: Snapshot,
    config: OdometryConfig,
) -> Result<(BodyDelta, Duration), Fault> {
    let dt = next
        .time
        .checked_sub(previous.time)
        .filter(|gap| !gap.is_zero() && *gap <= config.max_gap)
        .ok_or(Fault::Time)?;
    let seconds = dt.as_secs_f64();
    let limit =
        (config.max_linear_speed + config.max_angular_speed * config.track_width / 2.0) * seconds;
    let travel = |before: Option<f64>, after: Option<f64>, max: f64| {
        let change = after? - before?;
        (change.is_finite() && change.abs() <= max).then_some(change)
    };
    let [left, right] = [0, 1].map(|index| travel(previous.drive[index], next.drive[index], limit));
    let heading = previous
        .imu
        .zip(next.imu)
        .map(|(before, after)| wrap_heading(after - before))
        .filter(|angle| angle.abs() <= config.max_angular_speed * seconds)
        .or_else(|| {
            Some((right? - left?) / config.track_width)
                .filter(|angle| angle.abs() <= config.max_angular_speed * seconds)
        })
        .ok_or(Fault::Heading)?;
    let forward = travel(
        previous.forward,
        next.forward,
        (config.max_linear_speed + config.max_angular_speed * config.forward_offset.abs())
            * seconds,
    )
    .map(|distance| (distance, config.forward_offset))
    .or_else(|| match (left, right) {
        (Some(left), Some(right)) => Some(((left + right) * 0.5, 0.0)),
        (Some(left), None) => Some((left, config.track_width * 0.5)),
        (None, Some(right)) => Some((right, -config.track_width * 0.5)),
        _ => None,
    })
    .ok_or(Fault::Forward)?;
    let lateral = travel(
        previous.lateral,
        next.lateral,
        (config.max_linear_speed + config.max_angular_speed * config.lateral_offset.abs())
            * seconds,
    )
    .map(|distance| (distance, config.lateral_offset));
    if config.require_lateral && lateral.is_none() {
        return Err(Fault::Lateral);
    }
    Ok((body_delta(forward, lateral, heading), dt))
}
