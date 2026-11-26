//! Terminal pose settling shared by prepared-path followers.

use ozton_math::geometry::{Pose, bearing};

use super::*;

/// A terminal capture has exactly one active phase.
#[derive(Default)]
enum Phase {
    #[default]
    Acquire,
    Align,
    Approach,
    Brake(Duration),
    Heading,
}

/// Latch the final heading until it settles, even if residual translation
/// briefly takes the robot outside the position tolerance.
#[derive(Default)]
pub struct TerminalPose {
    phase: Phase,
    approach_reverse: Option<bool>,
}

impl TerminalPose {
    /// Whether the platform should brake instead of sending a zero-speed setpoint.
    pub const fn is_braking(&self) -> bool {
        matches!(self.phase, Phase::Brake(_))
    }

    /// Return wheel targets, full-pose readiness, progress objective and
    /// whether the position/heading phase changed.
    pub fn step(
        &mut self,
        controller: &mut DifferentialFeedback,
        target: Pose,
        reverse: bool,
        feedback: Feedback,
        dt: Duration,
        options: DriveOptions,
    ) -> ([f64; 2], bool, f64, bool) {
        let distance = feedback.pose.position.distance(target.position);
        let heading_error = wrap_heading(target.heading - feedback.pose.heading)
            .value
            .abs();
        let inside = distance <= options.position_tolerance.value;
        // Reserve some position tolerance for the final in-place turn. This
        // only tightens an already necessary point approach, not acceptance.
        let mut approach_options = options;
        if matches!(
            controller.config.path.terminal,
            crate::TerminalGuidance::Point
        ) {
            approach_options.position_tolerance.value *= 0.75;
        }
        let captured = distance <= approach_options.position_tolerance.value;
        let slow = feedback.forward_velocity.value.powi(2)
            + feedback.lateral_velocity.value.powi(2)
            <= options.speed_tolerance.value.powi(2)
            && feedback.angular_velocity.value.abs() <= options.angular_speed_tolerance.value;
        let mut changed = false;
        let alignment_tolerance = controller
            .config
            .path
            .terminal_alignment_tolerance
            .unwrap_or(options.heading_tolerance)
            .max(options.heading_tolerance);
        // A point terminal uses the conventional rotate-drive-rotate
        // sequence. Brake before changing the motion primitive.
        if matches!(self.phase, Phase::Acquire) {
            if inside && slow && heading_error <= options.heading_tolerance.value {
                self.phase = Phase::Heading;
                controller.reset();
                return ([0.0; 2], true, distance, true);
            }
            if !slow || inside {
                self.phase = Phase::Brake(Duration::ZERO);
                controller.reset();
                changed = true;
            }
        }
        if matches!(self.phase, Phase::Approach) && captured {
            self.phase = Phase::Brake(Duration::ZERO);
            controller.reset();
            changed = true;
        }
        if let Phase::Brake(ref mut stationary) = self.phase {
            *stationary = if slow {
                stationary.saturating_add(dt)
            } else {
                Duration::ZERO
            };
            if *stationary < options.settle.max(dt) {
                return ([0.0; 2], false, distance, changed);
            }
            self.phase = if inside {
                Phase::Heading
            } else {
                Phase::Acquire
            };
            controller.reset();
            changed = true;
        }
        if matches!(self.phase, Phase::Acquire) {
            let error = wrap_heading(
                bearing(target.position - feedback.pose.position) - feedback.pose.heading,
            )
            .value;
            let backing = error.abs() > core::f64::consts::FRAC_PI_2;
            self.approach_reverse = Some(backing);
            let travel_error = if backing {
                core::f64::consts::PI - error.abs()
            } else {
                error.abs()
            };
            self.phase = if matches!(
                controller.config.path.terminal,
                crate::TerminalGuidance::Point
            ) && travel_error > alignment_tolerance.value
            {
                Phase::Align
            } else {
                Phase::Approach
            };
            controller.reset();
            changed = true;
        }
        if matches!(self.phase, Phase::Align) {
            if inside {
                self.phase = Phase::Brake(Duration::ZERO);
                controller.reset();
                return ([0.0; 2], false, distance, true);
            }
            let heading = bearing(target.position - feedback.pose.position)
                + ozton_math::qty!((if self.approach_reverse.unwrap_or(reverse) { core::f64::consts::PI } else { 0.0 }) rad);
            let error = wrap_heading(heading - feedback.pose.heading).value.abs();
            let mut align_options = options;
            align_options.heading_tolerance = alignment_tolerance;
            let (wheels, ready) = controller.step(
                DifferentialTarget::Turn { heading },
                feedback,
                dt,
                align_options,
            );
            if ready {
                self.phase = Phase::Approach;
                controller.reset();
                changed = true;
            }
            return (
                wheels,
                false,
                options.track_width.value * error * 0.5,
                changed,
            );
        }
        let turning = matches!(self.phase, Phase::Heading);
        let command = if turning {
            DifferentialTarget::Turn {
                heading: target.heading,
            }
        } else {
            DifferentialTarget::DriveToPoint {
                target: target.position,
                reverse: self.approach_reverse.unwrap_or(reverse),
                maximum_speed: options.maximum_wheel_speed,
            }
        };
        let (wheels, ready) = match (turning, controller.config.path.terminal) {
            (
                false,
                crate::TerminalGuidance::Polar {
                    k_rho,
                    k_alpha,
                    k_beta,
                },
            ) => {
                let backing = self.approach_reverse.unwrap_or(reverse);
                let max_speed = stoppable_speed(
                    distance,
                    controller.config.limits.brake_latency.as_secs_f64(),
                    controller
                        .config
                        .limits
                        .forward_braking
                        .unwrap_or(controller.config.limits.deceleration)
                        .value,
                )
                .unwrap_or(0.0);
                let [linear, angular] =
                    ozton_math::following::PolarPose::new(k_rho, k_alpha, k_beta)
                        .unwrap()
                        .calculate(feedback.pose, target, backing, max_speed);
                let desired = wheel_speeds(
                    Velocity::new::<meter_per_second>(linear),
                    AngularVelocity::new::<radian_per_second>(angular),
                    options.track_width,
                )
                .map(|speed| speed.value);
                controller.limit_step(desired, false, dt, options)
            }
            _ => controller.step(
                command,
                feedback,
                dt,
                if turning { options } else { approach_options },
            ),
        };
        let objective = if turning {
            options.track_width.value * heading_error * 0.5
        } else {
            distance
        };
        // Reapproach only after the heading correction actually stops.
        if turning && ready && !inside {
            self.phase = Phase::Brake(Duration::ZERO);
            controller.reset();
            return ([0.0; 2], false, distance, true);
        }
        (wheels, turning && ready && inside, objective, changed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn already_settled_pose_does_not_require_a_second_dwell() {
        let options =
            DriveOptions::differential(ozton_math::qty!(0.33 m), ozton_math::qty!(0.5 m/s));
        let tick = Duration::from_millis(10);
        let mut terminal = TerminalPose::default();
        let mut controller = DifferentialFeedback::default();
        let (wheels, ready, _, _) = terminal.step(
            &mut controller,
            Pose::default(),
            false,
            Feedback {
                time: tick,
                pose: Pose::default(),
                forward_travel: Length::ZERO,
                forward_velocity: Velocity::ZERO,
                lateral_velocity: Velocity::ZERO,
                angular_velocity: AngularVelocity::ZERO,
                ranges: [None; _],
                reset_accepted: false,
            },
            tick,
            options,
        );
        assert!(ready);
        assert_eq!(wheels, [0.0; 2]);
        assert!(!terminal.is_braking());
    }

    #[test]
    fn final_heading_does_not_chase_bearing_when_position_drifts() {
        let options = DriveOptions::differential(
            ozton_math::qty!(0.3292677692 m),
            ozton_math::qty!(0.45 m/s),
        );
        let options = options.with_settling(
            ozton_math::qty!(0.75 in),
            ozton_math::qty!(2 deg),
            options.speed_tolerance,
            options.angular_speed_tolerance,
            options.settle,
        );
        let mut terminal = TerminalPose::default();
        let mut controller = DifferentialFeedback::default();
        let tick = Duration::from_millis(10);
        let feedback = |x, heading| Feedback {
            time: tick,
            pose: Pose::new(DVec2::new(x, 0.0), ozton_math::qty!((heading) rad)),
            forward_travel: Length::ZERO,
            forward_velocity: Velocity::ZERO,
            lateral_velocity: Velocity::ZERO,
            angular_velocity: AngularVelocity::ZERO,
            ranges: [None; _],
            reset_accepted: false,
        };
        let target = Pose::default();
        for _ in 0..(options.settle.as_millis() / tick.as_millis() - 1) {
            let (wheels, ready, _, _) = terminal.step(
                &mut controller,
                target,
                false,
                feedback(0.005, 0.2),
                tick,
                options,
            );
            assert!(!ready && terminal.is_braking());
            assert_eq!(wheels, [0.0; 2]);
        }
        let (_, ready, _, changed) = terminal.step(
            &mut controller,
            target,
            false,
            feedback(0.005, 0.2),
            tick,
            options,
        );
        assert!(changed && !ready && matches!(terminal.phase, Phase::Heading));

        let (wheels, ready, _, changed) = terminal.step(
            &mut controller,
            target,
            false,
            feedback(0.025, 0.2),
            tick,
            options,
        );
        assert!(!changed && !ready && matches!(terminal.phase, Phase::Heading));
        assert!(wheels[0] > wheels[1], "{wheels:?}");

        let (_, ready, _, changed) = terminal.step(
            &mut controller,
            target,
            false,
            feedback(0.025, 0.0),
            tick,
            options,
        );
        assert!(
            changed && !ready && !matches!(terminal.phase, Phase::Heading) && terminal.is_braking()
        );
    }

    #[test]
    fn terminal_capture_brakes_before_large_bearing_correction() {
        let options =
            DriveOptions::differential(ozton_math::qty!(0.33 m), ozton_math::qty!(0.5 m/s));
        let tick = Duration::from_millis(10);
        let mut terminal = TerminalPose::default();
        let mut controller = DifferentialFeedback::default();
        let target = Pose::new(DVec2::new(0.8382, 0.635), ozton_math::qty!(0 rad));
        let feedback = Feedback {
            time: tick,
            pose: Pose::new(DVec2::new(0.910, 0.615), ozton_math::qty!(0.418 rad)),
            forward_travel: Length::ZERO,
            forward_velocity: ozton_math::qty!(0.20 m/s),
            lateral_velocity: Velocity::ZERO,
            angular_velocity: ozton_math::qty!(0 rad/s),
            ranges: [None; _],
            reset_accepted: false,
        };
        let (wheels, ready, _, _) =
            terminal.step(&mut controller, target, false, feedback, tick, options);
        assert!(!ready && terminal.is_braking());
        assert_eq!(wheels, [0.0; 2]);
    }

    #[test]
    fn large_terminal_bearing_uses_stationary_turn() {
        let options =
            DriveOptions::differential(ozton_math::qty!(0.33 m), ozton_math::qty!(0.5 m/s));
        let tick = Duration::from_millis(10);
        let mut terminal = TerminalPose::default();
        let mut controller = DifferentialFeedback::default();
        let feedback = Feedback {
            time: tick,
            pose: Pose::new(DVec2::new(0.1, 0.01), ozton_math::qty!(0 rad)),
            forward_travel: Length::ZERO,
            forward_velocity: Velocity::ZERO,
            lateral_velocity: Velocity::ZERO,
            angular_velocity: AngularVelocity::ZERO,
            ranges: [None; _],
            reset_accepted: false,
        };
        let (wheels, ready, _, changed) = terminal.step(
            &mut controller,
            Pose::default(),
            false,
            feedback,
            tick,
            options,
        );
        assert!(changed && !ready && matches!(terminal.phase, Phase::Align));
        assert!(wheels[0].abs() > 0.0);
        assert!((wheels[0] + wheels[1]).abs() < 1e-9, "{wheels:?}");
    }

    #[test]
    fn initial_terminal_overshoot_uses_reverse_without_turning_around() {
        let options =
            DriveOptions::differential(ozton_math::qty!(0.33 m), ozton_math::qty!(0.5 m/s));
        let tick = Duration::from_millis(10);
        let mut terminal = TerminalPose::default();
        let mut controller = DifferentialFeedback::default();
        let feedback = Feedback {
            time: tick,
            pose: Pose::new(DVec2::new(0.0, 0.06), ozton_math::qty!(0 rad)),
            forward_travel: Length::ZERO,
            forward_velocity: Velocity::ZERO,
            lateral_velocity: Velocity::ZERO,
            angular_velocity: AngularVelocity::ZERO,
            ranges: [None; _],
            reset_accepted: false,
        };
        let (wheels, ready, _, _) = terminal.step(
            &mut controller,
            Pose::default(),
            false,
            feedback,
            tick,
            options,
        );
        assert!(!ready);
        assert!(wheels.iter().all(|speed| *speed < 0.0), "{wheels:?}");
    }

    #[test]
    fn capture_brakes_through_overshoot_before_reapproach() {
        let options = DriveOptions::differential(
            ozton_math::qty!(0.3292677692 m),
            ozton_math::qty!(0.45 m/s),
        );
        let tick = Duration::from_millis(10);
        let mut terminal = TerminalPose::default();
        let mut controller = DifferentialFeedback::default();
        let target = Pose::default();
        let feedback = |y, velocity| Feedback {
            time: tick,
            pose: Pose::new(DVec2::new(0.0, y), ozton_math::qty!(0 rad)),
            forward_travel: Length::ZERO,
            forward_velocity: ozton_math::qty!((velocity) m/s),
            lateral_velocity: Velocity::ZERO,
            angular_velocity: AngularVelocity::ZERO,
            ranges: [None; _],
            reset_accepted: false,
        };
        for sample in [feedback(-0.005, 0.3), feedback(0.04, 0.2)] {
            let (wheels, ready, _, _) =
                terminal.step(&mut controller, target, false, sample, tick, options);
            assert!(!ready && terminal.is_braking());
            assert_eq!(wheels, [0.0; 2]);
        }
        for _ in 0..(options.settle.as_millis() / tick.as_millis() - 1) {
            let (wheels, ready, _, _) = terminal.step(
                &mut controller,
                target,
                false,
                feedback(0.04, 0.0),
                tick,
                options,
            );
            assert!(!ready && terminal.is_braking());
            assert_eq!(wheels, [0.0; 2]);
        }
        let (wheels, ready, _, changed) = terminal.step(
            &mut controller,
            target,
            false,
            feedback(0.04, 0.0),
            tick,
            options,
        );
        assert!(changed && !ready && !terminal.is_braking());
        assert!(wheels.iter().all(|speed| *speed < 0.0), "{wheels:?}");
    }
}
