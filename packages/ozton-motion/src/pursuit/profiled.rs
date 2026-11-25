//! Spatial path-tangent feedback with a host-generated velocity envelope.

use core::f64::consts::PI;

use ozton_math::{
    control::Pid,
    geometry::{Pose, wheel_speeds, wrap_heading},
    path::{Path, PathSample, progress::Progress},
    uom::si::{
        angular_velocity::radian_per_second,
        f64::{AngularVelocity, Length, Velocity},
        velocity::meter_per_second,
    },
};

/// Progress and nominal remaining duration for nonblocking mechanism actions.
pub(crate) struct PathObservation {
    pub distance: f64,
    pub cross_track: f64,
    pub remaining_time: f64,
    pub nominal_progress_time: f64,
}

/// The reference is determined solely by measured spatial progress.
pub(crate) struct ProfiledFollower<'path> {
    progress: Progress<'path>,
    geometric: PathSample,
    lookahead: Length,
    yaw: Pid,
}

impl<'path> ProfiledFollower<'path> {
    pub fn new(path: &'path Path, lookahead: Length, mut yaw: Pid) -> Self {
        yaw.reset();
        Self {
            progress: Progress::new(path),
            geometric: path.start(),
            lookahead,
            yaw,
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
            nominal_progress_time: self.geometric.time,
        }
    }

    pub fn reset_feedback(&mut self) {
        self.yaw.reset();
    }

    pub fn is_terminal(&self, position_tolerance: f64) -> bool {
        self.progress.distance() >= self.progress.path().end().distance - position_tolerance
    }

    /// Wheel speeds in metres per second before shared physical constraints.
    pub fn command(
        &mut self,
        pose: Pose,
        reverse: bool,
        measured_speed: Velocity,
        measured_yaw: AngularVelocity,
        brake_latency: core::time::Duration,
        dt: core::time::Duration,
    ) -> [f64; 2] {
        let path = self.progress.path();
        let facing = ozton_math::qty!((if reverse { PI } else { 0.0 }) rad);
        let limits = path.limits();
        let remaining = (path.end().distance - self.geometric.distance).max(0.0);
        // The path tangent supplies the nominal heading. Correct lateral
        // deviation over a speed-dependent spatial horizon without cutting
        // across the next curve to chase a point.
        let reaction = brake_latency.as_secs_f64();
        let lookahead = self.lookahead.value.max(measured_speed.value.abs() * reaction * 2.0);
        let cross = self.geometric.pose.to_local(pose.position).x;
        let desired_heading =
            self.geometric.pose.heading + facing + ozton_math::qty!((cross.atan2(lookahead)) rad);
        let error = wrap_heading(desired_heading - pose.heading).value;

        // Preview the velocity envelope to launch from rest and brake before
        // changes in curvature. The measured speed determines actuator lead.
        let preview = (lookahead * 0.25).min(0.025).min(remaining * 0.5);
        let delay = (measured_speed.value.abs() * reaction).min(remaining * 0.5);
        let speed = path
            .at_distance(ozton_math::qty!((self.geometric.distance + preview) m))
            .speed
            .min(
                path.at_distance(
                    ozton_math::qty!((self.geometric.distance + preview.max(delay)) m),
                )
                .speed,
            )
            * error.cos().max(0.0);
        let curvature = path
            .at_distance(ozton_math::qty!((self.geometric.distance + delay) m))
            .curvature;
        let feedforward = speed * curvature;
        let angular = feedforward
            + self.yaw.update_with_rate(
                error,
                feedforward - measured_yaw.value,
                dt,
                core::time::Duration::ZERO,
                0.0,
            );
        let linear = if reverse { -speed } else { speed };
        wheel_speeds(
            Velocity::new::<meter_per_second>(linear),
            AngularVelocity::new::<radian_per_second>(angular),
            limits.track_width,
        )
        .map(|speed| speed.value)
    }
}

#[cfg(all(test, not(target_os = "vexos")))]
mod tests {
    use ozton_math::{
        geometry::DVec2,
        path::{Limits, Waypoint},
        qty,
    };

    use super::*;

    #[test]
    fn measured_yaw_damps_steering_before_heading_overshoots() {
        let limits = Limits::from_units(
            qty!(0.33 m),
            qty!(1 m/s),
            qty!(1 m/s),
            qty!(1 m/s^2),
            qty!(1 m/s^2),
            qty!(1 m/s^2),
        );
        let path = Path::new(
            &[
                Waypoint::new(DVec2::ZERO, 1.0),
                Waypoint::new(DVec2::Y, 1.0),
            ],
            limits,
        )
        .unwrap();
        let mut follower =
            ProfiledFollower::new(&path, qty!(0.15 m), Pid::new(4.0, 0.0, 0.4, None));
        let pose = Pose::new(DVec2::new(0.035, 0.2), qty!(0 rad));
        follower.observe(pose, qty!(0.3 m));
        let latency = core::time::Duration::from_millis(100);
        let undamped = follower.command(
            pose,
            false,
            qty!(0.3 m/s),
            qty!(0 rad/s),
            latency,
            core::time::Duration::from_millis(10),
        );
        let damped = follower.command(
            pose,
            false,
            qty!(0.3 m/s),
            qty!(2 rad/s),
            latency,
            core::time::Duration::from_millis(10),
        );
        assert!(undamped[1] > undamped[0]);
        assert!((damped[1] - damped[0]).abs() < (undamped[1] - undamped[0]).abs());
    }

    #[test]
    fn lateral_correction_has_same_world_turn_sign_in_reverse() {
        let limits = Limits::from_units(
            qty!(0.33 m),
            qty!(1 m/s),
            qty!(1 m/s),
            qty!(1 m/s^2),
            qty!(1 m/s^2),
            qty!(1 m/s^2),
        );
        let path = Path::new(
            &[
                Waypoint::new(DVec2::ZERO, 1.0),
                Waypoint::new(DVec2::Y, 1.0),
            ],
            limits,
        )
        .unwrap();
        let mut follower =
            ProfiledFollower::new(&path, qty!(0.19 m), Pid::new(4.0, 0.0, 0.4, None));
        let pose = Pose::new(DVec2::new(0.035, 0.2), qty!(0 rad));
        follower.observe(pose, qty!(0.3 m));
        let latency = core::time::Duration::from_millis(120);
        let dt = core::time::Duration::from_millis(10);
        let forward = follower.command(pose, false, qty!(0.3 m/s), qty!(0 rad/s), latency, dt);
        follower.reset_feedback();
        let reverse = follower.command(
            Pose {
                heading: qty!(180 deg),
                ..pose
            },
            true,
            qty!(-0.3 m/s),
            qty!(0 rad/s),
            latency,
            dt,
        );
        assert!(forward[1] > forward[0]);
        assert!(reverse[1] > reverse[0]);
        assert!(forward[0] + forward[1] > 0.0);
        assert!(reverse[0] + reverse[1] < 0.0);
    }

    #[test]
    fn stationary_reference_does_not_advance_and_can_start_in_both_directions() {
        let limits = Limits::from_units(
            qty!(0.33 m),
            qty!(1 m/s),
            qty!(1 m/s),
            qty!(1 m/s^2),
            qty!(1 m/s^2),
            qty!(1 m/s^2),
        );
        let path = Path::new(
            &[
                Waypoint::new(DVec2::ZERO, 1.0),
                Waypoint::new(DVec2::Y, 1.0),
            ],
            limits,
        )
        .unwrap();
        let mut follower =
            ProfiledFollower::new(&path, qty!(0.1 m), Pid::new(4.0, 0.0, 0.4, None));
        let pose = path.start().pose;
        let initial = follower.observe(pose, qty!(0.04 m));
        for _ in 0..100 {
            let current = follower.observe(pose, qty!(0.04 m));
            assert_eq!(current.distance, initial.distance);
            assert_eq!(current.nominal_progress_time, initial.nominal_progress_time);
        }
        let latency = core::time::Duration::from_millis(100);
        let wheels = follower.command(
            pose,
            false,
            qty!(0 m/s),
            qty!(0 rad/s),
            latency,
            core::time::Duration::from_millis(10),
        );
        assert!(wheels.iter().all(|v| *v > 0.0));
        let reverse = Pose {
            heading: pose.heading + qty!(180 deg),
            ..pose
        };
        let wheels = follower.command(
            reverse,
            true,
            qty!(0 m/s),
            qty!(0 rad/s),
            latency,
            core::time::Duration::from_millis(10),
        );
        assert!(wheels.iter().all(|v| *v < 0.0));
        assert!(!follower.is_terminal(0.02));
        let moved = follower.observe(Pose::new(DVec2::new(0.0, 0.03), pose.heading), qty!(0.04 m));
        assert!(moved.distance > initial.distance);
        assert!(moved.remaining_time < initial.remaining_time);
    }
}
