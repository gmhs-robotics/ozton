//! Pure pursuit and RAMSETE tracking for the same prepared path.

use core::{f64::consts::PI, time::Duration};
use ozton_math::{
    desaturate,
    following::{PurePursuit, Ramsete},
    geometry::{Pose, wheel_speeds},
    path::{Path, PathSample, progress::Progress},
    uom::si::{
        angular_velocity::radian_per_second,
        f64::{AngularVelocity, Length, Time, Velocity},
        time::second,
        velocity::meter_per_second,
    },
};

/// Select the path-tracking algorithm and its tuning.
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

/// One trajectory observation, shared by progress monitoring and path actions.
pub(crate) struct PathObservation {
    pub distance: f64,
    pub cross_track: f64,
    pub remaining_time: f64,
    pub reference_time: f64,
}

/// Owns geometric progress and the bounded time reference for one execution.
/// The session is responsible for actuator output, settling and safety.
pub(crate) struct ProfiledFollower<'path> {
    follower: Follower,
    progress: Progress<'path>,
    geometric: PathSample,
    reference_time: f64,
}

impl<'path> ProfiledFollower<'path> {
    pub fn new(path: &'path Path, follower: Follower) -> Self {
        Self {
            follower,
            progress: Progress::new(path),
            geometric: path.start(),
            reference_time: 0.0,
        }
    }

    pub fn observe(
        &mut self,
        pose: Pose,
        dt: Duration,
        advance_limit: Length,
        lead: Length,
    ) -> PathObservation {
        self.progress.update(pose.position, advance_limit);
        let path = self.progress.path();
        let distance = self.progress.distance();
        self.geometric = path.at_distance(ozton_math::qty!((distance) m));
        let end = path.end();
        // Bound the nominal reference by a physical lead over measured progress.
        let limit = path
            .at_distance(ozton_math::qty!((distance + lead.value) m))
            .time;
        self.reference_time = (self.reference_time + dt.as_secs_f64())
            .min(limit)
            .max(self.reference_time)
            .min(end.time);
        PathObservation {
            distance,
            cross_track: self.geometric.pose.position.distance(pose.position),
            remaining_time: (end.time - self.geometric.time).max(0.0),
            reference_time: self.reference_time,
        }
    }

    pub fn is_terminal(&self, position_tolerance: f64) -> bool {
        let end = self.progress.path().end();
        match self.follower {
            Follower::PurePursuit { .. } => {
                self.progress.distance() >= end.distance - position_tolerance
            }
            Follower::Ramsete { .. } => self.reference_time >= end.time,
        }
    }

    /// Wheel speeds in metres per second, before the shared command limiter.
    pub fn command(&self, pose: Pose, dt: Duration, reverse: bool) -> [f64; 2] {
        let path = self.progress.path();
        let dt = dt.as_secs_f64();
        let facing = ozton_math::qty!((if reverse { PI } else { 0.0 }) rad);
        let limits = path.limits();
        let (linear, angular) = match self.follower {
            Follower::PurePursuit { lookahead } => {
                let virtual_pose = Pose {
                    heading: pose.heading + facing,
                    ..pose
                };
                let target = virtual_pose.to_local(self.progress.lookahead(pose.position, lookahead.value));
                if target.y < 0.0 {
                    (0.0, 2.0 * ozton_math::geometry::bearing(target).value)
                } else {
                    let speed = path
                        .at_time(Time::new::<second>(self.geometric.time + dt))
                        .speed;
                    PurePursuit::new().twist(
                        pose,
                        virtual_pose.transform_point(target),
                        Velocity::new::<meter_per_second>(speed),
                        reverse,
                    )
                }
            }
            Follower::Ramsete { b, zeta } => {
                Ramsete { b, zeta }.twist(pose, path.at_time(Time::new::<second>(self.reference_time)), reverse)
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
}
