//! Pure pursuit and RAMSETE tracking for the same prepared path.

use core::f64::consts::PI;

pub use ozton_math::path::progress::Progress;
use ozton_math::{
    desaturate,
    following::{PurePursuit, Ramsete},
    geometry::{Pose, wheel_speeds, wrap_heading},
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

/// Advance a monotonic moving reference. Limit lead while far from the endpoint,
/// but release the time gate inside the terminal neighborhood: otherwise a
/// zero-speed reference can approach the final time asymptotically and stall.
pub fn advance_reference_time(
    previous: f64,
    dt: f64,
    geometric: ozton_math::path::PathSample,
    end: ozton_math::path::PathSample,
    terminal_distance: f64,
) -> f64 {
    let next = previous + dt;
    let limit = if end.distance - geometric.distance <= terminal_distance {
        end.time
    } else {
        geometric.time + 0.25
    };
    next.min(limit).max(previous).min(end.time)
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
        Follower::PurePursuit { .. } => {
            progress.distance() >= end.distance - position_tolerance.max(0.025)
        }
        Follower::Ramsete { .. } => sample.elapsed.value + sample.period.value >= end.time,
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
    position_tolerance: Length,
) -> [f64; 2] {
    let path = progress.path();
    let pose = sample.pose;
    let (elapsed, dt, position_tolerance) = (
        sample.elapsed.value,
        sample.period.value,
        position_tolerance.value,
    );
    let facing = ozton_math::qty!((if reverse { PI } else { 0.0 }) rad);
    let end = path.end();
    let limits = path.limits();
    let (linear, angular) = if is_terminal(progress, follower, sample, position_tolerance) {
        let error = pose.to_local(end.pose.position);
        let heading_error = wrap_heading(end.pose.heading + facing - pose.heading).value;
        let distance = error.length();
        if distance > position_tolerance {
            let travel_heading = ozton_math::geometry::bearing(error);
            let backward = travel_heading.value.abs() > PI / 2.0;
            let bearing = wrap_heading(
                travel_heading + ozton_math::qty!((if backward { PI } else { 0.0 }) rad),
            )
            .value;
            let terminal_speed = (2.0 * distance).min(limits.linear_speed.value * 0.4);
            (
                terminal_speed * bearing.cos() * if backward { -1.0 } else { 1.0 },
                3.0 * bearing,
            )
        } else {
            (0.0, 3.0 * heading_error)
        }
    } else {
        match follower {
            Follower::PurePursuit { lookahead } => {
                let virtual_pose = Pose {
                    heading: pose.heading + facing,
                    ..pose
                };
                let target =
                    virtual_pose.to_local(progress.lookahead(pose.position, lookahead.value));
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
            Follower::Ramsete { b, zeta } => Ramsete { b, zeta }.twist(
                pose,
                path.at_time(Time::new::<second>(elapsed + dt)),
                reverse,
            ),
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
