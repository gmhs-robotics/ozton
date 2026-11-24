//! Terminal pose settling shared by prepared-path followers.

use ozton_math::geometry::{Pose, bearing};

use super::*;

/// Latch the final heading until it settles, even if residual translation
/// briefly takes the robot outside the position tolerance.
#[derive(Default)]
pub struct TerminalPose {
    turning: bool,
    braking: bool,
    stationary: Duration,
    approach_reverse: Option<bool>,
}

impl TerminalPose {
    /// Whether the platform should brake instead of sending a zero-speed setpoint.
    pub const fn is_braking(&self) -> bool {
        self.braking
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
        let mut changed = false;
        if !self.turning && inside && !self.braking {
            self.braking = true;
            self.stationary = Duration::ZERO;
            controller.reset();
            changed = true;
        }
        if self.braking {
            let slow = feedback.forward_velocity.value.powi(2)
                + feedback.lateral_velocity.value.powi(2)
                <= options.speed_tolerance.value.powi(2)
                && feedback.angular_velocity.value.abs() <= options.angular_speed_tolerance.value;
            self.stationary = if slow {
                self.stationary.saturating_add(dt)
            } else {
                Duration::ZERO
            };
            if self.stationary < options.settle.max(dt) {
                return ([0.0; 2], false, distance, changed);
            }
            self.braking = false;
            self.turning = inside;
            // Recover a short overshoot in the closer travel direction, but
            // preserve the planned direction farther from the endpoint.
            self.approach_reverse = if !inside
                && distance
                    <= controller.config.approach.reversal_distance_factor * options.position_tolerance.value
            {
                let error = wrap_heading(
                    bearing(target.position - feedback.pose.position) - feedback.pose.heading,
                );
                Some(error.value.abs() > core::f64::consts::FRAC_PI_2)
            } else {
                None
            };
            controller.reset();
            controller.point_backing = None;
            controller.point_aligning = false;
            changed = true;
        }

        let command = if self.turning {
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
        let (wheels, ready) = controller.step(command, feedback, dt, options);
        let objective = if self.turning {
            options.track_width.value * heading_error * 0.5
        } else {
            distance
        };

        // A heading correction can shift the chassis. Reapproach only after
        // the turn has actually stopped, not on each tolerance crossing.
        if self.turning && ready && !inside {
            self.turning = false;
            self.braking = true;
            self.stationary = Duration::ZERO;
            controller.reset();
            return ([0.0; 2], false, distance, true);
        }
        (wheels, self.turning && ready && inside, objective, changed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        assert!(changed && !ready && terminal.turning);

        let (wheels, ready, _, changed) = terminal.step(
            &mut controller,
            target,
            false,
            feedback(0.025, 0.2),
            tick,
            options,
        );
        assert!(!changed && !ready && terminal.turning);
        assert!(wheels[0] > wheels[1], "{wheels:?}");

        let (_, ready, _, changed) = terminal.step(
            &mut controller,
            target,
            false,
            feedback(0.025, 0.0),
            tick,
            options,
        );
        assert!(changed && !ready && !terminal.turning && terminal.is_braking());
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
