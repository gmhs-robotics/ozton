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
}

impl<'path> ProfiledFollower<'path> {
    pub fn new(path: &'path Path, lookahead: Length) -> Self {
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
            nominal_progress_time: self.geometric.time,
        }
    }

    pub fn is_terminal(&self, position_tolerance: f64) -> bool {
        self.progress.distance() >= self.progress.path().end().distance - position_tolerance
    }

    /// Wheel speeds in metres per second before shared physical constraints.
    pub fn command(
        &self,
        pose: Pose,
        reverse: bool,
        measured_speed: Velocity,
        measured_yaw: AngularVelocity,
        brake_latency: core::time::Duration,
    ) -> [f64; 2] {
        let path = self.progress.path();
        let facing = ozton_math::qty!((if reverse { PI } else { 0.0 }) rad);
        let limits = path.limits();
        let remaining = (path.end().distance - self.geometric.distance).max(0.0);
        // Use a spatial carrot, independent of the number of samples per metre.
        // Increase the preview with speed so that a fast straight does not chase
        // centimetre-scale localization corrections.
        let reaction = brake_latency.as_secs_f64();
        let lookahead = self.lookahead.value.max(measured_speed.value.abs() * reaction * 2.0);
        let point = path
            .at_distance(ozton_math::qty!((self.geometric.distance + lookahead) m))
            .pose
            .position;
        // Predict heading through the actuator delay. This is equivalent to
        // damping heading error by measured yaw, without differentiating MCL.
        let travel = Pose {
            heading: pose.heading + facing
                + ozton_math::qty!((measured_yaw.value * reaction) rad),
            ..pose
        };
        let target = travel.to_local(point);
        let bearing = ozton_math::geometry::bearing(target).value;
        let (linear, angular) = if target.y <= 0.0 {
            (0.0, 2.0 * bearing)
        } else {
            let preview = (lookahead * 0.25).min(0.025).min(remaining * 0.5);
            let speed = path
                .at_distance(ozton_math::qty!((self.geometric.distance + preview) m))
                .speed;
            let delay = (measured_speed.value.abs() * reaction).min(remaining * 0.5);
            let speed = speed.min(
                path.at_distance(
                    ozton_math::qty!((self.geometric.distance + preview.max(delay)) m),
                )
                .speed,
            );
            // Do not drive rapidly toward a carrot far from the travel heading.
            // Preserve curvature while reducing the speed for recovery.
            let speed = speed * bearing.cos().max(0.0);
            PurePursuit::new().twist(
                Pose {
                    heading: travel.heading - facing,
                    ..pose
                },
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
        let mut follower = ProfiledFollower::new(&path, qty!(0.15 m));
        let pose = Pose::new(DVec2::new(0.035, 0.2), qty!(0 rad));
        follower.observe(pose, qty!(0.3 m));
        let latency = core::time::Duration::from_millis(100);
        let undamped = follower.command(pose, false, qty!(0.3 m/s), qty!(0 rad/s), latency);
        let damped = follower.command(pose, false, qty!(0.3 m/s), qty!(2 rad/s), latency);
        assert!(undamped[1] > undamped[0]);
        assert!((damped[1] - damped[0]).abs() < (undamped[1] - undamped[0]).abs());
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
        let mut follower = ProfiledFollower::new(&path, qty!(0.1 m));
        let pose = path.start().pose;
        let initial = follower.observe(pose, qty!(0.04 m));
        for _ in 0..100 {
            let current = follower.observe(pose, qty!(0.04 m));
            assert_eq!(current.distance, initial.distance);
            assert_eq!(current.nominal_progress_time, initial.nominal_progress_time);
        }
        let latency = core::time::Duration::from_millis(100);
        let wheels = follower.command(pose, false, qty!(0 m/s), qty!(0 rad/s), latency);
        assert!(wheels.iter().all(|v| *v > 0.0));
        let reverse = Pose {
            heading: pose.heading + qty!(180 deg),
            ..pose
        };
        let wheels = follower.command(reverse, true, qty!(0 m/s), qty!(0 rad/s), latency);
        assert!(wheels.iter().all(|v| *v < 0.0));
        assert!(!follower.is_terminal(0.02));
        let moved = follower.observe(Pose::new(DVec2::new(0.0, 0.03), pose.heading), qty!(0.04 m));
        assert!(moved.distance > initial.distance);
        assert!(moved.remaining_time < initial.remaining_time);
    }
}
