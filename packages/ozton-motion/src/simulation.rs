//! Host-only, telemetry-calibrated motion simulation. No firmware dependencies.
use std::{collections::VecDeque, time::Duration};

use ozton_localization::Feedback;
use ozton_math::{
    geometry::{BodyDelta, DVec2, Pose, wrap_heading},
    path::Path,
    qty,
};

use crate::{
    MotionConfig,
    basic::differential::{DifferentialFeedback, TerminalPose},
    pursuit::profiled::ProfiledFollower,
};

/// Effective command-to-feedback response, fitted from logged wheel targets.
#[derive(Clone, Copy, Debug)]
pub struct AxisResponse {
    pub delay: Duration,
    pub time_constant: Duration,
    pub gain: f64,
    pub cross_gain: f64,
    pub bias: f64,
}

impl AxisResponse {
    fn valid(self) -> bool {
        !self.time_constant.is_zero()
            && [self.gain, self.cross_gain, self.bias]
                .iter()
                .all(|v| v.is_finite())
            && self.gain > 0.0
    }

    fn step(self, current: f64, own: f64, other: f64, dt: Duration) -> f64 {
        let alpha = 1.0 - (-dt.as_secs_f64() / self.time_constant.as_secs_f64()).exp();
        // The fitted moving-response bias is not a force at rest. A zero
        // wheel command must decay toward rest instead of drifting indefinitely.
        let target = if own == 0.0 && other == 0.0 {
            0.0
        } else {
            self.gain * own + self.cross_gain * other + self.bias
        };
        current + alpha * (target - current)
    }
}

/// Independent forward and yaw response, with optional cross coupling.
#[derive(Clone, Copy, Debug)]
pub struct ChassisResponse {
    pub forward: AxisResponse,
    pub yaw: AxisResponse,
}

/// Effective lateral feedback coupled to yaw, identified from telemetry.
/// This may include tracking-reference error as well as actual chassis slip.
/// It is not a calibration of the geometric pivot.
#[derive(Clone, Copy, Debug, Default)]
pub struct LateralResponse {
    pub yaw_gain_m: f64,
    pub cornering_gain_s: f64,
}

impl LateralResponse {
    fn velocity(self, forward: f64, yaw: f64) -> f64 {
        (self.yaw_gain_m + self.cornering_gain_s * forward.abs()) * yaw
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SimulationReport {
    pub elapsed: Duration,
    pub settled: bool,
    pub progress_m: f64,
    pub cross_track_rms_m: f64,
    pub maximum_cross_track_m: f64,
    pub final_position_error_m: f64,
    pub final_heading_error_rad: f64,
    pub absolute_rotation_rad: f64,
    pub first_clearance_violation: Option<Duration>,
}

/// Run the production follower, wheel limiter and terminal controller against
/// an identified effective response. This is not a rigid-body or MCL model.
/// The clearance predicate receives every integrated pose, including intermediate
/// swept poses. Violations are recorded, not used to alter the controller.
pub fn simulate_path(
    path: &Path,
    config: MotionConfig,
    response: ChassisResponse,
    initial_pose: Pose,
    reverse: bool,
    maximum_time: Duration,
    clear: impl FnMut(Pose) -> bool,
) -> Option<SimulationReport> {
    simulate_path_with_lateral(
        path,
        config,
        response,
        LateralResponse::default(),
        initial_pose,
        reverse,
        maximum_time,
        clear,
    )
}

/// Simulate with an independently identified lateral response rather than
/// assuming perfect no-slip differential-drive motion.
#[allow(clippy::too_many_arguments)]
pub fn simulate_path_with_lateral(
    path: &Path,
    config: MotionConfig,
    response: ChassisResponse,
    lateral: LateralResponse,
    initial_pose: Pose,
    reverse: bool,
    maximum_time: Duration,
    mut clear: impl FnMut(Pose) -> bool,
) -> Option<SimulationReport> {
    let limits = path.limits();
    let (config, options) = config.constrained_for_path(config.options(limits.track_width), limits);
    if !config.valid()
        || !options.valid()
        || !initial_pose.is_finite()
        || !response.forward.valid()
        || !response.yaw.valid()
        || !lateral.yaw_gain_m.is_finite()
        || !lateral.cornering_gain_s.is_finite()
        || maximum_time.is_zero()
    {
        return None;
    }
    let tick = config.timing.period;
    let dt = tick.as_secs_f64();
    let delay_steps = |axis: AxisResponse| (axis.delay.as_secs_f64() / dt).round() as usize;
    let delays = [delay_steps(response.forward), delay_steps(response.yaw)];
    let capacity = *delays.iter().max()? + 1;
    if capacity > 10_000 {
        return None;
    }
    let mut commands = VecDeque::from(vec![[0.0; 2]; capacity]);
    let mut follower = ProfiledFollower::new(path, config.path.lookahead, config.tuning.angular)
        .with_guidance(config.path.guidance);
    let mut controller = DifferentialFeedback::new(config.tuning).with_config(config);
    let mut terminal_pose = TerminalPose::default();
    let mut terminal = false;
    let mut pose = initial_pose;
    let end = path.end().pose;
    let target = Pose::new(
        end.position,
        end.heading + qty!((if reverse { 180.0 } else { 0.0 }) deg),
    );
    let mut elapsed = Duration::ZERO;
    let mut settled_time = Duration::ZERO;
    let mut velocity = 0.0;
    let mut yaw = 0.0;
    let mut travel = 0.0;
    let mut rotation = 0.0;
    let mut squared_cross = 0.0;
    let mut max_cross: f64 = 0.0;
    let mut samples = 0_u32;
    let mut progress = 0.0;
    let mut first_violation = (!clear(pose)).then_some(elapsed);

    while elapsed < maximum_time {
        let observation =
            follower.observe(pose, qty!((options.maximum_wheel_speed.value * dt * 2.0) m));
        progress = observation.distance;
        if !terminal && follower.is_terminal(options.position_tolerance.value) {
            terminal = true;
            controller.reset_feedback();
        }
        let feedback = Feedback {
            time: elapsed,
            pose,
            forward_travel: qty!((travel) m),
            forward_velocity: qty!((velocity) m/s),
            lateral_velocity: qty!((lateral.velocity(velocity, yaw)) m/s),
            angular_velocity: qty!((yaw) rad/s),
            ranges: [None; _],
            reset_accepted: false,
        };
        let (wheels, ready) = if terminal {
            let (wheels, ready, _, _) =
                terminal_pose.step(&mut controller, target, reverse, feedback, tick, options);
            (wheels, ready)
        } else {
            samples += 1;
            squared_cross += observation.cross_track.powi(2);
            max_cross = max_cross.max(observation.cross_track);
            let desired = follower.command(
                pose,
                reverse,
                [feedback.lateral_velocity, feedback.forward_velocity],
                feedback.angular_velocity,
                config.limits.brake_latency,
                tick,
            );
            controller.limit_step(desired, false, tick, options)
        };
        settled_time = if ready {
            settled_time.saturating_add(tick)
        } else {
            Duration::ZERO
        };
        if settled_time >= options.settle.max(tick) {
            break;
        }

        // The fitted moving-phase response approximates zero-speed braking too;
        // a separately identified brake-mode response can replace it later.
        let wheels = if terminal && (terminal_pose.is_braking() || ready) {
            [0.0; 2]
        } else {
            wheels
        };
        controller.record_applied_wheels(wheels);
        let _ = commands.pop_back();
        commands.push_front([
            (wheels[0] + wheels[1]) * 0.5,
            (wheels[1] - wheels[0]) / limits.track_width.value,
        ]);
        let forward = commands[delays[0]];
        let angular = commands[delays[1]];
        let next_v = response
            .forward
            .step(velocity, forward[0], forward[1], tick);
        let next_w = response.yaw.step(yaw, angular[1], angular[0], tick);
        let ds = (velocity + next_v) * 0.5 * dt;
        let dh = (yaw + next_w) * 0.5 * dt;
        let lateral_travel =
            (lateral.velocity(velocity, yaw) + lateral.velocity(next_v, next_w)) * 0.5 * dt;
        let previous = pose;
        pose = pose.integrate(BodyDelta {
            translation: DVec2::new(lateral_travel, ds),
            heading: qty!((dh) rad),
        });
        velocity = next_v;
        yaw = next_w;
        travel += ds;
        rotation += dh.abs();
        elapsed += tick;
        if first_violation.is_none()
            && (1..=4).any(|i| !clear(previous.interpolate(pose, f64::from(i) / 4.0)))
        {
            first_violation = Some(elapsed);
        }
    }

    Some(SimulationReport {
        elapsed,
        settled: settled_time >= options.settle.max(tick),
        progress_m: progress,
        cross_track_rms_m: (squared_cross / f64::from(samples.max(1))).sqrt(),
        maximum_cross_track_m: max_cross,
        final_position_error_m: pose.position.distance(target.position),
        final_heading_error_rad: wrap_heading(target.heading - pose.heading).value.abs(),
        absolute_rotation_rad: rotation,
        first_clearance_violation: first_violation,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lateral_response_reverses_with_yaw_and_vanishes_on_straights() {
        let lateral = LateralResponse {
            yaw_gain_m: 0.05,
            cornering_gain_s: 0.01,
        };
        assert_eq!(lateral.velocity(1.0, 0.0), 0.0);
        assert!((lateral.velocity(1.0, 2.0) - 0.12).abs() < 1e-12);
        assert!((lateral.velocity(-1.0, -2.0) + 0.12).abs() < 1e-12);
    }

    #[test]
    fn zero_wheel_command_does_not_create_motion_from_bias() {
        let axis = AxisResponse {
            delay: Duration::ZERO,
            time_constant: Duration::from_millis(100),
            gain: 1.0,
            cross_gain: 0.0,
            bias: -0.1,
        };
        let dt = Duration::from_millis(10);
        assert_eq!(axis.step(0.0, 0.0, 0.0, dt), 0.0);
        assert!(axis.step(0.2, 0.0, 0.0, dt) > 0.0);
        assert!(axis.step(0.2, 0.0, 0.0, dt) < 0.2);
        assert!(axis.step(0.0, 0.0, 0.5, dt) < 0.0);
    }

    #[test]
    fn response_uses_exact_discrete_first_order_step() {
        let axis = AxisResponse {
            delay: Duration::ZERO,
            time_constant: Duration::from_millis(100),
            gain: 1.0,
            cross_gain: 0.0,
            bias: 0.0,
        };
        let value = axis.step(0.0, 1.0, 0.0, Duration::from_millis(100));
        assert!((value - (1.0 - (-1.0_f64).exp())).abs() < 1e-12);
    }
}
