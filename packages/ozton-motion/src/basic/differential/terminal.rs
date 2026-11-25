//! Terminal pose settling shared by prepared-path followers.

use ozton_math::geometry::{Pose, bearing};

use super::*;

/// A terminal capture has exactly one active phase.
#[derive(Default)]
enum Phase {
    #[default]
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
        let slow = feedback.forward_velocity.value.powi(2)
            + feedback.lateral_velocity.value.powi(2)
            <= options.speed_tolerance.value.powi(2)
            && feedback.angular_velocity.value.abs() <= options.angular_speed_tolerance.value;
        let mut changed = false;
        // An already settled pose needs only the caller's arrival dwell.
        if matches!(self.phase, Phase::Approach)
            && inside
            && slow
            && heading_error <= options.heading_tolerance.value
        {
            self.phase = Phase::Heading;
            controller.reset();
            return ([0.0; 2], true, distance, true);
        }
        // On entering terminal capture, do not carry forward momentum into a
        // point correction that requires a substantial change of direction.
        // Once stationary, pick the nearer travel direction independently of
        // the direction used on the authored path.
        if matches!(self.phase, Phase::Approach) && self.approach_reverse.is_none() {
            let bearing_error = wrap_heading(
                bearing(target.position - feedback.pose.position) - feedback.pose.heading,
            )
            .value;
            let backing = bearing_error.abs() > core::f64::consts::FRAC_PI_2;
            let travel_error = if backing {
                core::f64::consts::PI - bearing_error.abs()
            } else {
                bearing_error.abs()
            };
            if !slow
                && (travel_error > core::f64::consts::FRAC_PI_4
                    || (feedback.forward_velocity.value < 0.0) != backing)
            {
                self.phase = Phase::Brake(Duration::ZERO);
                controller.reset();
                changed = true;
            } else {
                self.approach_reverse = Some(if inside { reverse } else { backing });
                controller.point_backing = None;
            }
        }
        if matches!(self.phase, Phase::Approach) && inside {
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
                Phase::Approach
            };
            // Recover short overshoots in the closer travel direction.
            self.approach_reverse = if !inside {
                let error = wrap_heading(
                    bearing(target.position - feedback.pose.position) - feedback.pose.heading,
                );
                Some(error.value.abs() > core::f64::consts::FRAC_PI_2)
            } else {
                None
            };
            controller.reset();
            controller.point_backing = None;
            changed = true;
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
                let facing = ozton_math::qty!((if backing { 180.0 } else { 0.0 }) deg);
                let direction = bearing(target.position - feedback.pose.position);
                let alpha = wrap_heading(direction - feedback.pose.heading - facing).value;
                let beta = wrap_heading(target.heading + facing - direction).value;
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
                let speed = (k_rho * distance).min(max_speed) * alpha.cos().max(0.0);
                let linear = if backing { -speed } else { speed };
                let angular = k_alpha * alpha + k_beta * beta;
                let desired = wheel_speeds(
                    Velocity::new::<meter_per_second>(linear),
                    AngularVelocity::new::<radian_per_second>(angular),
                    options.track_width,
                )
                .map(|speed| speed.value);
                controller.limit_step(desired, false, dt, options)
            }
            _ => controller.step(command, feedback, dt, options),
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
