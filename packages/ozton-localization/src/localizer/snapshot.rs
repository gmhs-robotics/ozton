//! Timestamped device snapshots and travel source selection.

use std::time::Duration;

pub use ozton_math::localization::observation::Snapshot;
use ozton_math::{
    geometry::{BodyDelta, wrap_heading},
    localization::odometry::body_delta,
    uom::si::f64::{Angle, Length},
};

#[derive(Debug, Clone, Copy)]
pub struct OdometryConfig {
    /// A positive width lets the left/right encoder difference estimate yaw.
    pub track_width: Length,
    /// X-right position of a forward wheel from The Origin.
    pub forward_offset: Length,
    /// Y-forward position of a right-positive lateral wheel from The Origin.
    pub lateral_offset: Length,
    pub max_gap: Duration,
}

impl OdometryConfig {
    pub const fn for_mounted_wheels(max_gap: Duration) -> Self {
        use ozton_math::uom::ConstZero;
        Self::from_units(Length::ZERO, Length::ZERO, Length::ZERO, max_gap)
    }

    pub const fn from_units(
        track_width: Length,
        forward_offset: Length,
        lateral_offset: Length,
        max_gap: Duration,
    ) -> Self {
        Self {
            track_width,
            forward_offset,
            lateral_offset,
            max_gap,
        }
    }

    pub fn is_valid(self) -> bool {
        self.track_width.value.is_finite()
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

pub(super) fn interval(
    previous: Snapshot,
    next: Snapshot,
    config: OdometryConfig,
) -> Result<Duration, Fault> {
    next.time
        .checked_sub(previous.time)
        .filter(|gap| !gap.is_zero() && *gap <= config.max_gap)
        .ok_or(Fault::Time)
}

pub(crate) fn delta(
    previous: Snapshot,
    next: Snapshot,
    config: OdometryConfig,
) -> Result<(BodyDelta, Duration), Fault> {
    let dt = interval(previous, next, config)?;
    let travel = |before: Option<Length>, after: Option<Length>| {
        let change = after?.value - before?.value;
        change.is_finite().then_some(change)
    };
    let [left, right] = [0, 1].map(|index| travel(previous.drive[index], next.drive[index]));
    let heading = previous
        .imu
        .zip(next.imu)
        .map(|(before, after)| wrap_heading(after - before))
        .filter(|angle| angle.is_finite())
        .or_else(|| {
            (config.track_width.value > 0.0)
                .then(|| {
                    Some(ozton_math::qty!(((right? - left?) / config.track_width.value) rad))
                })
                .flatten()
                .filter(|angle| angle.is_finite())
        });
    let forward = travel(previous.forward, next.forward)
        .map(|distance| (distance, config.forward_offset.value))
        .or(match (left, right) {
            (Some(left), Some(right)) => Some(((left + right) * 0.5, 0.0)),
            (Some(left), None) => Some((left, -config.track_width.value * 0.5)),
            (None, Some(right)) => Some((right, config.track_width.value * 0.5)),
            _ => None,
        });
    let lateral = travel(previous.lateral, next.lateral)
        .map(|distance| (distance, config.lateral_offset.value));
    if heading.is_none() && forward.is_none() && lateral.is_none() {
        return Ok((
            BodyDelta {
                translation: glam::DVec2::ZERO,
                heading: Angle::default(),
            },
            dt,
        ));
    }
    let heading = heading.unwrap_or_default();
    let forward = forward.unwrap_or_default();
    let lengths = |(travel, offset)| {
        (
            ozton_math::qty!((travel) m),
            ozton_math::qty!((offset) m),
        )
    };
    Ok((
        body_delta(lengths(forward), lateral.map(lengths), heading),
        dt,
    ))
}
