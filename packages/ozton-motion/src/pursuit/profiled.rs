//! Spatial pure pursuit with a host-generated velocity envelope.

use core::f64::consts::PI;

use ozton_math::{
    following::PurePursuit,
    geometry::{Pose, wheel_speeds},
    path::{Path, PathSample, progress::Progress},
    uom::si::{
        angular_velocity::radian_per_second,
        f64::{AngularVelocity, Length, Velocity},
        velocity::meter_per_second,
    },
};

/// The spatial pursuit controller's geometric lookahead.
#[derive(Debug, Clone, Copy)]
pub enum Follower {
    PurePursuit { lookahead: Length },
    // Kept only until consumers migrate to the single spatial follower.
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

/// Progress and nominal remaining duration for nonblocking mechanism actions.
pub(crate) struct PathObservation {
    pub distance: f64,
    pub cross_track: f64,
    pub remaining_time: f64,
    pub reference_time: f64,
}

/// The reference is determined solely by measured spatial progress.
pub(crate) struct ProfiledFollower<'path> {
    progress: Progress<'path>,
    geometric: PathSample,
    lookahead: Length,
}

impl<'path> ProfiledFollower<'path> {
    pub fn new(path: &'path Path, follower: Follower, default_lookahead: Length) -> Self {
        let lookahead = match follower {
            Follower::PurePursuit { lookahead } => lookahead,
            Follower::Ramsete { .. } => default_lookahead,
        };
        Self {
            progress: Progress::new(path),
            geometric: path.start(),
            lookahead,
        }
    }

    pub fn observe(&mut self, pose: Pose, advance_limit: Length) -> PathObservation {
        self.progress.update(pose.position, advance_limit);
        let path = self.progress.path();
        let distance = self.progress.distance();
        self.geometric = path.at_distance(ozton_math::qty!((distance) m));
        PathObservation {
            distance,
            cross_track: self.geometric.pose.position.distance(pose.position),
            remaining_time: (path.end().time - self.geometric.time).max(0.0),
            reference_time: self.geometric.time,
        }
    }

    pub fn is_terminal(&self, position_tolerance: f64) -> bool {
        self.progress.distance() >= self.progress.path().end().distance - position_tolerance
    }

    /// Wheel speeds in metres per second before shared physical constraints.
    pub fn command(&self, pose: Pose, reverse: bool) -> [f64; 2] {
        let path = self.progress.path();
        let facing = ozton_math::qty!((if reverse { PI } else { 0.0 }) rad);
        let limits = path.limits();
        let travel = Pose {
            heading: pose.heading + facing,
            ..pose
        };
        let point = self.progress.lookahead(pose.position, self.lookahead.value);
        let target = travel.to_local(point);
        let (linear, angular) = if target.y <= 0.0 {
            (0.0, 2.0 * ozton_math::geometry::bearing(target).value)
        } else {
            // A short spatial preview starts the robot from a zero-speed
            // profile endpoint and begins braking ahead of corners.
            let remaining = (path.end().distance - self.geometric.distance).max(0.0);
            let preview = (self.lookahead.value * 0.25).min(0.025).min(remaining * 0.5);
            let speed = path
                .at_distance(ozton_math::qty!((self.geometric.distance + preview) m))
                .speed;
            PurePursuit::new().twist(
                pose,
                point,
                Velocity::new::<meter_per_second>(speed),
                reverse,
            )
        };
        wheel_speeds(
            Velocity::new::<meter_per_second>(linear),
            AngularVelocity::new::<radian_per_second>(angular),
            limits.track_width,
        )
        .map(|speed| speed.value)
    }
}

/// Bound lateral acceleration of a differential wheel command while preserving
/// curvature. The same bound is needed after independent linear/yaw slew.
pub(crate) fn limit_lateral_acceleration(
    wheels: [f64; 2],
    track_width: f64,
    maximum: f64,
) -> [f64; 2] {
    let linear = (wheels[0] + wheels[1]) * 0.5;
    let angular = (wheels[1] - wheels[0]) / track_width;
    let lateral = (linear * angular).abs();
    if lateral <= maximum {
        return wheels;
    }
    let scale = (maximum / lateral).sqrt();
    wheels.map(|speed| speed * scale)
}

#[cfg(test)]
mod tests {
    use super::limit_lateral_acceleration;

    #[test]
    fn lateral_limit_preserves_curvature_and_turn_direction() {
        let width = 0.33;
        let maximum = 0.55;
        for wheels in [[0.4, 1.4], [1.4, 0.4], [-0.4, -1.4], [-1.4, -0.4]] {
            let limited = limit_lateral_acceleration(wheels, width, maximum);
            let v = (limited[0] + limited[1]) * 0.5;
            let w = (limited[1] - limited[0]) / width;
            assert!(((v * w).abs() - maximum).abs() < 1e-12);
            assert!((limited[0] * wheels[1] - limited[1] * wheels[0]).abs() < 1e-12);
        }
    }

    #[test]
    fn lateral_limit_leaves_straight_and_stationary_turn_commands_unchanged() {
        for wheels in [[1.0, 1.0], [-1.0, -1.0], [-0.5, 0.5], [0.0, 0.0]] {
            assert_eq!(limit_lateral_acceleration(wheels, 0.33, 0.55), wheels);
        }
    }
}
