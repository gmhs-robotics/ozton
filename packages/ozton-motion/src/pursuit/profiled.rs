//! Pure pursuit and RAMSETE tracking for the same prepared path.

use core::f64::consts::PI;

pub use ozton_math::path::progress::Progress;
use ozton_math::{
    desaturate,
    following::{PurePursuit, Ramsete},
    geometry::{Pose, wheel_speeds},
    uom::si::{
        angular_velocity::radian_per_second,
        f64::{AngularVelocity, Length, Time, Velocity},
        time::second,
        velocity::meter_per_second,
    },
};

#[derive(Debug, Clone, Copy)]
pub enum Follower {
    PurePursuit { lookahead: Length },
    Ramsete { b: f64, zeta: f64 },
}

impl Follower {
    pub fn is_valid(self) -> bool {
        match self {
            Self::PurePursuit { lookahead } => lookahead.value.is_finite() && lookahead.value > 0.0,
            Self::Ramsete { b, zeta } => {
                b.is_finite() && b > 0.0 && (0.0..=1.0).contains(&zeta) && zeta > 0.0
            }
        }
    }
}

/// Advance nominal trajectory time while bounding the reference's arc-length
/// lead over observed progress. The lead is a physical distance, not a
/// fixed time lag that changes its meaning with the profile speed.
pub fn advance_reference_time(
    previous: f64,
    dt: f64,
    progress: &Progress<'_>,
    lead: Length,
) -> f64 {
    let path = progress.path();
    let limit = path
        .at_distance(ozton_math::qty!((progress.distance() + lead.value) m))
        .time;
    (previous + dt)
        .min(limit)
        .max(previous)
        .min(path.end().time)
}

/// Whether to stop using the moving reference and settle at the final pose.
pub fn is_terminal(
    progress: &Progress<'_>,
    follower: Follower,
    sample: FollowerSample,
    position_tolerance: f64,
) -> bool {
    let end = progress.path().end();
    match follower {
        Follower::PurePursuit { .. } => progress.distance() >= end.distance - position_tolerance,
        Follower::Ramsete { .. } => sample.elapsed.value >= end.time,
    }
}

/// Timing and pose for one follower tick.
#[derive(Clone, Copy, Debug)]
pub struct FollowerSample {
    pub pose: Pose,
    pub elapsed: Time,
    pub period: Time,
}

/// Wheel speeds in meters per second. The caller owns settling and stopping.
pub fn command(
    follower: Follower,
    progress: &Progress<'_>,
    sample: FollowerSample,
    reverse: bool,
) -> [f64; 2] {
    let path = progress.path();
    let pose = sample.pose;
    let (elapsed, dt) = (sample.elapsed.value, sample.period.value);
    let facing = ozton_math::qty!((if reverse { PI } else { 0.0 }) rad);
    let limits = path.limits();
    let (linear, angular) = match follower {
        Follower::PurePursuit { lookahead } => {
            let virtual_pose = Pose {
                heading: pose.heading + facing,
                ..pose
            };
            let target = virtual_pose.to_local(progress.lookahead(pose.position, lookahead.value));
            if target.y < 0.0 {
                (0.0, 2.0 * ozton_math::geometry::bearing(target).value)
            } else {
                let reference = path.at_distance(ozton_math::qty!((progress.distance()) m));
                let speed = path.at_time(Time::new::<second>(reference.time + dt)).speed;
                PurePursuit::new().twist(
                    pose,
                    virtual_pose.transform_point(target),
                    Velocity::new::<meter_per_second>(speed),
                    reverse,
                )
            }
        }
        Follower::Ramsete { b, zeta } => {
            Ramsete { b, zeta }.twist(pose, path.at_time(Time::new::<second>(elapsed)), reverse)
        }
    };
    desaturate(
        wheel_speeds(
            Velocity::new::<meter_per_second>(linear),
            AngularVelocity::new::<radian_per_second>(angular),
            limits.track_width,
        )
        .map(|speed| speed.value),
        limits.wheel_speed.value,
    )
}
