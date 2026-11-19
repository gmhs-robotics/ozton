//! Timestamped device snapshots and travel source selection.

use std::time::Duration;

use ozton_math::{
    geometry::{BodyDelta, Pose, wrap_heading},
    localization::odometry::body_delta,
    uom::si::f64::{Angle, AngularVelocity, Length, Velocity},
};

/// A GPS fix and its estimated position error, in meters.
#[derive(Debug, Clone, Copy)]
pub struct GpsFix {
    pub pose: Pose,
    pub position_error: Length,
    pub timestamp: Duration,
}

/// One tick's cumulative encoder positions in meters and heading in radians.
/// Unavailable or uncalibrated devices supply None.
#[derive(Debug, Clone, Copy, Default)]
pub struct Snapshot {
    pub time: Duration,
    pub drive: [Option<Length>; 2],
    pub forward: Option<Length>,
    pub lateral: Option<Length>,
    pub imu: Option<Angle>,
    pub gps: Option<GpsFix>,
}

#[derive(Debug, Clone, Copy)]
pub struct OdometryConfig {
    /// Zero requires IMU heading and both drive encoders; positive enables wheel fallback.
    pub track_width: Length,
    /// X-right position of a forward wheel from The Origin.
    pub forward_offset: Length,
    /// Y-forward position of a right-positive lateral wheel from The Origin.
    pub lateral_offset: Length,
    /// Require lateral feedback, e.g. on a holonomic drivetrain.
    pub require_lateral: bool,
    pub max_linear_speed: Velocity,
    pub max_angular_speed: AngularVelocity,
    pub max_gap: Duration,
}

impl OdometryConfig {
    /// Creates odometry settings from physical lengths, speeds, and the sample gap.
    pub const fn from_units(
        track_width: ozton_math::uom::si::f64::Length,
        forward_offset: ozton_math::uom::si::f64::Length,
        lateral_offset: ozton_math::uom::si::f64::Length,
        require_lateral: bool,
        max_linear_speed: ozton_math::uom::si::f64::Velocity,
        max_angular_speed: ozton_math::uom::si::f64::AngularVelocity,
        max_gap: Duration,
    ) -> Self {
        Self {
            track_width,
            forward_offset,
            lateral_offset,
            require_lateral,
            max_linear_speed,
            max_angular_speed,
            max_gap,
        }
    }

    pub fn is_valid(self) -> bool {
        [self.max_linear_speed.value, self.max_angular_speed.value]
            .iter()
            .all(|value| value.is_finite() && *value > 0.0)
            && self.track_width.value.is_finite()
            && self.track_width.value >= 0.0
            && self.forward_offset.value.is_finite()
            && self.lateral_offset.value.is_finite()
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
    let limit = (config.max_linear_speed.value
        + config.max_angular_speed.value * config.track_width.value / 2.0)
        * seconds;
    let travel = |before: Option<Length>, after: Option<Length>, max: f64| {
        let change = after?.value - before?.value;
        (change.is_finite() && change.abs() <= max).then_some(change)
    };
    let [left, right] = [0, 1].map(|index| travel(previous.drive[index], next.drive[index], limit));
    let heading = previous
        .imu
        .zip(next.imu)
        .map(|(before, after)| wrap_heading(ozton_math::si(after.value - before.value)))
        .filter(|angle| angle.abs() <= config.max_angular_speed.value * seconds)
        .or_else(|| {
            if config.track_width.value == 0.0 {
                return None;
            }
            Some((right? - left?) / config.track_width.value)
                .filter(|angle| angle.abs() <= config.max_angular_speed.value * seconds)
        })
        .ok_or(Fault::Heading)?;
    let forward = travel(
        previous.forward,
        next.forward,
        (config.max_linear_speed.value
            + config.max_angular_speed.value * config.forward_offset.value.abs())
            * seconds,
    )
    .map(|distance| (distance, config.forward_offset.value))
    .or_else(|| match (left, right) {
        (Some(left), Some(right)) => Some(((left + right) * 0.5, 0.0)),
        (Some(left), None) if config.track_width.value > 0.0 => {
            Some((left, -config.track_width.value * 0.5))
        }
        (None, Some(right)) if config.track_width.value > 0.0 => {
            Some((right, config.track_width.value * 0.5))
        }
        _ => None,
    })
    .ok_or(Fault::Forward)?;
    let lateral = travel(
        previous.lateral,
        next.lateral,
        (config.max_linear_speed.value
            + config.max_angular_speed.value * config.lateral_offset.value.abs())
            * seconds,
    )
    .map(|distance| (distance, config.lateral_offset.value));
    if config.require_lateral && lateral.is_none() {
        return Err(Fault::Lateral);
    }
    let lengths = |(travel, offset)| (ozton_math::si(travel), ozton_math::si(offset));
    Ok((
        body_delta(
            lengths(forward),
            lateral.map(lengths),
            ozton_math::si(heading),
        ),
        dt,
    ))
}
