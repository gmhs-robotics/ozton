//! Geometric pure pursuit and time-indexed trajectory tracking.

use core::f64::consts::PI;

use ozton_math::{
    control::Pid,
    following::{LqrSchedule, PurePursuit},
    geometry::{BodyDelta, DVec2, Pose, bearing, pursuit_curvature, wheel_speeds, wrap_heading},
    path::{Path, PathSample, braking::stoppable_speed, progress::Progress},
    uom::si::{
        angular_velocity::radian_per_second,
        f64::{AngularVelocity, Length, Velocity},
        velocity::meter_per_second,
    },
};

use super::unicycle::Guidance;

/// Progress and nominal remaining duration for nonblocking mechanism actions.
pub(crate) struct PathObservation {
    pub distance: f64,
    pub cross_track: f64,
    pub remaining_time: f64,
    pub nominal_progress_time: f64,
}

/// Pure pursuit uses spatial progress; trajectory controllers use elapsed time.
pub(crate) struct ProfiledFollower<'path> {
    progress: Progress<'path>,
    geometric: PathSample,
    lookahead: Length,
    yaw: Pid,
    guidance: Guidance,
    prediction_latency: Option<[core::time::Duration; 2]>,
    adaptive_lookahead: Option<core::time::Duration>,
    heading_recovery: bool,
    elapsed: f64,
    reference_speed_scale: f64,
    lqr: Option<LqrSchedule>,
}

impl<'path> ProfiledFollower<'path> {
    pub fn new(path: &'path Path, lookahead: Length, mut yaw: Pid) -> Self {
        yaw.reset();
        Self {
            progress: Progress::new(path),
            geometric: path.start(),
            lookahead,
            yaw,
            guidance: Guidance::PurePursuit,
            prediction_latency: None,
            adaptive_lookahead: None,
            heading_recovery: false,
            elapsed: 0.0,
            reference_speed_scale: 1.0,
            lqr: None,
        }
    }

    #[cfg(test)]
    pub fn with_guidance(mut self, guidance: crate::PathGuidance) -> Self {
        #[allow(
            clippy::unwrap_used,
            reason = "Test fixtures supply valid guidance constants"
        )]
        let guidance = Guidance::new(guidance).unwrap();
        self.guidance = guidance;
        self
    }

    pub fn with_config(mut self, config: crate::PathConfig) -> Option<Self> {
        self.guidance = Guidance::new(config.guidance)?;
        self.reference_speed_scale = config.reference_speed_scale;
        self.prediction_latency = config.prediction_latency;
        self.adaptive_lookahead = config.adaptive_lookahead;
        self.heading_recovery = config.heading_recovery;
        Some(self)
    }

    pub fn prepare(mut self, period: core::time::Duration, reverse: bool) -> Option<Self> {
        let scale = self.reference_speed_scale;
        if !(0.1..=1.0).contains(&scale) {
            return None;
        }
        if let Guidance::Lqr(controller) = self.guidance {
            let path = self.progress.path();
            self.lqr =
                Some(
                    controller.prepare(path.end().time / scale, period.as_secs_f64(), |t| {
                        let sample = path.at_time(ozton_math::qty!((t * scale) s));
                        let v = if reverse { -sample.speed } else { sample.speed };
                        [v * scale, sample.speed * sample.curvature * scale]
                    })?,
                );
        }
        Some(self)
    }

    fn time_scale(&self) -> f64 {
        if matches!(self.guidance, Guidance::PurePursuit) {
            1.0
        } else {
            self.reference_speed_scale
        }
    }

    pub fn planned_duration(&self) -> f64 {
        self.progress.path().end().time / self.time_scale()
    }

    pub fn observe(&mut self, pose: Pose, advance_limit: Length) -> PathObservation {
        self.progress.update(pose.position, advance_limit);
        let path = self.progress.path();
        let distance = self.progress.distance();
        self.geometric = path.at_distance(ozton_math::qty!((distance) m));
        PathObservation {
            distance,
            cross_track: self.geometric.pose.position.distance(pose.position),
            remaining_time: (path.end().time - self.geometric.time).max(0.0) / self.time_scale(),
            nominal_progress_time: self.geometric.time,
        }
    }

    /// Initialize a new tracking law at the shared geometric progress.
    pub fn seek(&mut self, distance: f64) {
        self.progress.seek(distance);
        self.geometric = self
            .progress
            .path()
            .at_distance(ozton_math::qty!((distance) m));
        self.elapsed = self.geometric.time / self.time_scale();
    }

    pub fn reset_feedback(&mut self) {
        self.yaw.reset();
    }

    pub fn is_terminal(&self, position_tolerance: f64) -> bool {
        if matches!(self.guidance, Guidance::PurePursuit) {
            self.progress.distance() >= self.progress.path().end().distance - position_tolerance
        } else {
            self.elapsed >= self.planned_duration()
        }
    }

    /// Wheel speeds in metres per second before shared physical constraints.
    /// Measured body velocity is ordered as lateral (right), then forward.
    pub fn command(
        &mut self,
        pose: Pose,
        reverse: bool,
        measured_velocity: [Velocity; 2],
        measured_yaw: AngularVelocity,
        brake_latency: core::time::Duration,
        dt: core::time::Duration,
    ) -> [f64; 2] {
        let [measured_lateral, measured_speed] = measured_velocity;
        let path = self.progress.path();
        let facing = ozton_math::qty!((if reverse { PI } else { 0.0 }) rad);
        let limits = path.limits();
        let remaining = (path.end().distance - self.geometric.distance).max(0.0);
        // Trajectory controllers track a timed reference, not a nearest-point
        // projection. The geometric cursor remains available for mechanisms.
        if !matches!(self.guidance, Guidance::PurePursuit) {
            // Predict the pose and sample the reference at each actuator's
            // effective response time. Both outputs share the same trajectory
            // clock; a preview never advances that clock.
            let [forward_delay, yaw_delay] = self
                .prediction_latency
                .unwrap_or([core::time::Duration::ZERO; 2]);
            let scale = self.reference_speed_scale;
            let control_at = |delay: core::time::Duration| {
                let t = (self.elapsed + delay.as_secs_f64()).min(path.end().time / scale);
                let horizon = (t - self.elapsed).max(0.0);
                let predicted = pose.integrate(BodyDelta {
                    translation: DVec2::new(measured_lateral.value, measured_speed.value) * horizon,
                    heading: ozton_math::qty!((measured_yaw.value * horizon) rad),
                });
                let reference = path.at_time(ozton_math::qty!((t * scale) s));
                let local = predicted.to_local(reference.pose.position);
                let heading =
                    wrap_heading(reference.pose.heading + facing - predicted.heading).value;
                let velocity = if reverse {
                    -reference.speed
                } else {
                    reference.speed
                };
                let twist = [
                    velocity * scale,
                    reference.speed * reference.curvature * scale,
                ];
                let error = [local.y, -local.x, heading];
                match self.guidance {
                    Guidance::Ramsete(controller) => controller.calculate(error, twist),
                    Guidance::Lqr(_) => {
                        #[allow(
                            clippy::expect_used,
                            reason = "LQR preparation must succeed before trajectory output starts"
                        )]
                        let schedule = self.lqr.as_ref().expect("prepared LQR schedule");
                        schedule.calculate(t, error, twist)
                    }
                    Guidance::PurePursuit => unreachable!(),
                }
            };
            let [linear, mut angular] = control_at(forward_delay);
            if yaw_delay != forward_delay {
                angular = control_at(yaw_delay)[1];
            }
            self.elapsed += dt.as_secs_f64();
            return wheel_speeds(
                Velocity::new::<meter_per_second>(linear),
                AngularVelocity::new::<radian_per_second>(angular),
                limits.track_width,
            )
            .map(|speed| speed.value);
        }
        let [forward_delay, yaw_delay] = self
            .prediction_latency
            .unwrap_or([core::time::Duration::ZERO; 2]);
        let lookahead = self
            .adaptive_lookahead
            .map_or(self.lookahead.value, |horizon| {
                self.lookahead
                    .value
                    .max(measured_speed.value.abs() * horizon.as_secs_f64())
            });
        // Optional constant-twist prediction, independent of lookahead.
        let predicted = if yaw_delay.is_zero() {
            pose
        } else {
            let reaction = yaw_delay.as_secs_f64();
            pose.integrate(BodyDelta {
                translation: DVec2::new(measured_lateral.value, measured_speed.value) * reaction,
                heading: ozton_math::qty!((measured_yaw.value * reaction) rad),
            })
        };
        // Preview the profile by the distance travelled during actuator
        // response. From rest, use the constant-acceleration launch distance
        // instead of a fixed spatial offset. Keep the preview short of the
        // terminal zero-speed sample until measured progress reaches it.
        let horizon = forward_delay.as_secs_f64().max(dt.as_secs_f64());
        let launch = 0.5 * limits.acceleration.value * horizon * horizon;
        let preview = launch
            .max(measured_speed.value.abs() * horizon)
            .min(remaining * 0.5);
        // The spatial profile assumes immediate braking. Account for the
        // measured reaction interval before entering terminal pose capture.
        let endpoint_limit = stoppable_speed(
            remaining,
            horizon.max(brake_latency.as_secs_f64()),
            limits.deceleration.value,
        )
        .unwrap_or(0.0);
        let speed = path
            .at_distance(ozton_math::qty!((self.geometric.distance + preview) m))
            .speed
            .min(endpoint_limit);
        let travel_pose = Pose::new(predicted.position, predicted.heading - facing);
        let carrot = self.progress.lookahead(predicted.position, lookahead);
        let curvature = PurePursuit::new().forward_curvature(travel_pose, carrot);
        let curvature_speed = |curvature: f64| {
            // Limit speed on the actual pursuit arc, not only the authored
            // trajectory. Both bounds preserve geometric curvature.
            let wheel =
                limits.wheel_speed.value / (1.0 + limits.track_width.value * curvature.abs() * 0.5);
            let lateral = (limits.lateral_acceleration.value / curvature.abs()).sqrt();
            let speed = speed.min(wheel).min(lateral);
            (speed, speed * curvature)
        };
        let (speed, angular) = match curvature {
            Some(curvature) => curvature_speed(curvature),
            None if self.heading_recovery => {
                // If the carrot is behind the robot, turn in place to
                // reacquire it; do not substitute another path controller.
                let error = bearing(travel_pose.to_local(carrot)).value;
                let max_yaw = 2.0 * limits.wheel_speed.value / limits.track_width.value;
                let yaw = self.yaw.update_with_rate(
                    error,
                    -measured_yaw.value,
                    dt,
                    core::time::Duration::ZERO,
                    0.0,
                );
                (0.0, yaw.clamp(-max_yaw, max_yaw))
            }
            None => curvature_speed(pursuit_curvature(travel_pose.to_local(carrot))),
        };
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
    fn near_endpoint_pursuit_requests_feasible_wheels_in_both_directions() {
        let limits = Limits::from_units(
            qty!(0.33 m),
            qty!(1 m/s),
            qty!(1 m/s),
            qty!(1 m/s^2),
            qty!(1 m/s^2),
            qty!(0.4 m/s^2),
        );
        let path = Path::new(
            &[
                Waypoint::new(DVec2::ZERO, 1.0),
                Waypoint::new(DVec2::Y, 1.0),
            ],
            limits,
        )
        .unwrap();
        for reverse in [false, true] {
            let mut follower =
                ProfiledFollower::new(&path, qty!(0.27 m), Pid::new(3.0, 0.0, 0.4, None));
            follower.seek(0.95);
            let pose = Pose::new(
                DVec2::new(0.04, 0.95),
                qty!((if reverse { 180.0 } else { 0.0 }) deg),
            );
            let wheels = follower.command(
                pose,
                reverse,
                [qty!(0 m/s); 2],
                qty!(0 rad/s),
                core::time::Duration::from_millis(150),
                core::time::Duration::from_millis(10),
            );
            assert!(
                wheels
                    .iter()
                    .all(|v| v.abs() <= limits.wheel_speed.value + 1e-12)
            );
            let v = (wheels[0] + wheels[1]) * 0.5;
            let w = (wheels[1] - wheels[0]) / limits.track_width.value;
            assert!((v * w).abs() <= limits.lateral_acceleration.value + 1e-12);
            assert!(v.abs() > 0.0);
        }
    }

    #[test]
    fn predicted_yaw_changes_pursuit_curvature() {
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
        follower.prediction_latency = Some([core::time::Duration::from_millis(100); 2]);
        let pose = Pose::new(DVec2::new(0.035, 0.2), qty!(0 rad));
        follower.observe(pose, qty!(0.3 m));
        let latency = core::time::Duration::from_millis(100);
        let undamped = follower.command(
            pose,
            false,
            [qty!(0 m/s), qty!(0.3 m/s)],
            qty!(0 rad/s),
            latency,
            core::time::Duration::from_millis(10),
        );
        let damped = follower.command(
            pose,
            false,
            [qty!(0 m/s), qty!(0.3 m/s)],
            qty!(0.5 rad/s),
            latency,
            core::time::Duration::from_millis(10),
        );
        assert!(undamped[1] > undamped[0]);
        assert!((damped[1] - damped[0]).abs() < (undamped[1] - undamped[0]).abs());
    }

    #[test]
    fn lateral_slip_is_corrected_before_position_error_grows() {
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
            ProfiledFollower::new(&path, qty!(0.19 m), Pid::new(3.0, 0.0, 0.4, None));
        follower.prediction_latency = Some([core::time::Duration::from_millis(170); 2]);
        let pose = Pose::new(DVec2::new(0.0, 0.3), qty!(0 rad));
        follower.observe(pose, qty!(0.3 m));
        let wheels = follower.command(
            pose,
            false,
            [qty!(0.2 m/s), qty!(0.5 m/s)],
            qty!(0 rad/s),
            core::time::Duration::from_millis(170),
            core::time::Duration::from_millis(8),
        );
        assert!(wheels[1] > wheels[0], "{wheels:?}");
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
        let forward = follower.command(
            pose,
            false,
            [qty!(0 m/s), qty!(0.3 m/s)],
            qty!(0 rad/s),
            latency,
            dt,
        );
        follower.reset_feedback();
        let reverse = follower.command(
            Pose {
                heading: qty!(180 deg),
                ..pose
            },
            true,
            [qty!(0 m/s), qty!(-0.3 m/s)],
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
    fn pure_pursuit_corrects_world_lateral_error_in_both_directions() {
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
        for reverse in [false, true] {
            let heading = if reverse { 180.0 } else { 0.0 };
            let pose = Pose::new(DVec2::new(0.035, 0.2), qty!((heading) deg));
            let mut follower =
                ProfiledFollower::new(&path, qty!(0.2 m), Pid::new(4.0, 0.0, 0.4, None))
                    .with_guidance(crate::PathGuidance::PurePursuit);
            follower.observe(pose, qty!(0.3 m));
            let wheels = follower.command(
                pose,
                reverse,
                [qty!(0 m/s), qty!((if reverse { -0.3 } else { 0.3 }) m/s)],
                qty!(0 rad/s),
                core::time::Duration::from_millis(100),
                core::time::Duration::from_millis(10),
            );
            assert!(wheels[1] > wheels[0]);
            let speed = (wheels[0] + wheels[1]).abs() * 0.5;
            assert!(speed > 0.0);
            assert!((wheels[0] + wheels[1]) * if reverse { -1.0 } else { 1.0 } > 0.0);
            // For a straight path and lateral offset e, a radius L circle
            // gives signed curvature 2e/L², independently of the direction.
            let curvature = (wheels[1] - wheels[0]) / (limits.track_width.value * speed);
            assert!((curvature - 2.0 * 0.035 / 0.2_f64.powi(2)).abs() < 1e-9);
        }
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
        let mut follower = ProfiledFollower::new(&path, qty!(0.1 m), Pid::new(4.0, 0.0, 0.4, None));
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
            [qty!(0 m/s), qty!(0 m/s)],
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
            [qty!(0 m/s), qty!(0 m/s)],
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

    #[test]
    fn trajectory_reference_advances_with_time_even_without_spatial_progress() {
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
        let tick = core::time::Duration::from_millis(10);
        for guidance in [crate::PathGuidance::RAMSETE, crate::PathGuidance::LQR] {
            let mut follower =
                ProfiledFollower::new(&path, qty!(0.2 m), Pid::new(4.0, 0.0, 0.4, None))
                    .with_guidance(guidance)
                    .prepare(tick, false)
                    .unwrap();
            let pose = path.start().pose;
            for _ in 0..50 {
                let progress = follower.observe(pose, qty!(0.02 m));
                assert_eq!(progress.distance, 0.0);
                follower.command(
                    pose,
                    false,
                    [qty!(0 m/s), qty!(0 m/s)],
                    qty!(0 rad/s),
                    core::time::Duration::ZERO,
                    tick,
                );
            }
            assert!((follower.elapsed - 0.5).abs() < 1e-12);
            let wheels = follower.command(
                pose,
                false,
                [qty!(0 m/s), qty!(0 m/s)],
                qty!(0 rad/s),
                core::time::Duration::ZERO,
                tick,
            );
            assert!(wheels.iter().all(|v| *v > 0.0));
        }
    }

    #[test]
    fn timed_reference_scaling_preserves_geometry_and_retimes_twist() {
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
        let dt = core::time::Duration::from_millis(10);
        for guidance in [crate::PathGuidance::RAMSETE, crate::PathGuidance::LQR] {
            let mut follower =
                ProfiledFollower::new(&path, qty!(0.2 m), Pid::new(4.0, 0.0, 0.4, None))
                    .with_guidance(guidance);
            follower.reference_speed_scale = 0.8;
            let mut follower = follower.prepare(dt, false).unwrap();
            let t = path.end().time * 0.5;
            follower.elapsed = t / 0.8;
            let sample = path.at_time(qty!((t) s));
            let wheels = follower.command(
                sample.pose,
                false,
                [qty!(0 m/s); 2],
                qty!(0 rad/s),
                core::time::Duration::ZERO,
                dt,
            );
            assert!(
                wheels
                    .iter()
                    .all(|v| (*v - sample.speed * 0.8).abs() < 1e-9)
            );
            follower.elapsed = path.end().time;
            assert!(!follower.is_terminal(0.01));
            follower.elapsed = follower.planned_duration();
            assert!(follower.is_terminal(0.01));
        }
    }

    #[test]
    fn timed_yaw_preview_uses_its_own_latency() {
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
        let tick = core::time::Duration::from_millis(10);
        let forward_delay = core::time::Duration::from_millis(100);
        let pose = Pose::new(DVec2::new(0.04, 0.3), qty!(5 deg));
        for guidance in [crate::PathGuidance::RAMSETE, crate::PathGuidance::LQR] {
            let mut follower =
                ProfiledFollower::new(&path, qty!(0.2 m), Pid::new(4.0, 0.0, 0.4, None))
                    .with_guidance(guidance)
                    .prepare(tick, false)
                    .unwrap();
            let mut command = |yaw_delay| {
                follower.seek(0.3);
                follower.prediction_latency = Some([forward_delay, yaw_delay]);
                follower.command(
                    pose,
                    false,
                    [qty!(0 m/s), qty!(0.3 m/s)],
                    qty!(0.5 rad/s),
                    forward_delay,
                    tick,
                )
            };
            let a = command(forward_delay);
            let b = command(core::time::Duration::from_millis(180));
            assert!((a[0] + a[1] - b[0] - b[1]).abs() < 1e-12);
            assert!((a[1] - a[0] - b[1] + b[0]).abs() > 1e-5);
        }
    }

    #[test]
    fn timed_prediction_previews_reference_without_advancing_clock() {
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
        let tick = core::time::Duration::from_millis(10);
        let delay = core::time::Duration::from_millis(100);
        for guidance in [crate::PathGuidance::RAMSETE, crate::PathGuidance::LQR] {
            let mut follower =
                ProfiledFollower::new(&path, qty!(0.2 m), Pid::new(4.0, 0.0, 0.4, None))
                    .with_guidance(guidance)
                    .prepare(tick, false)
                    .unwrap();
            follower.prediction_latency = Some([delay; 2]);
            let wheels = follower.command(
                path.start().pose,
                false,
                [qty!(0 m/s), qty!(0 m/s)],
                qty!(0 rad/s),
                delay,
                tick,
            );
            assert!(wheels.iter().all(|v| *v > 0.0), "{wheels:?}");
            assert!((follower.elapsed - tick.as_secs_f64()).abs() < 1e-12);
        }
    }
}
