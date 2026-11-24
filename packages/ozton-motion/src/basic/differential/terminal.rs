//! Terminal pose settling shared by prepared-path followers.

use ozton_math::geometry::Pose;

use super::*;

/// Latch the final heading until it settles, even if residual translation
/// briefly takes the robot outside the position tolerance.
#[derive(Default)]
pub struct TerminalPose {
    turning: bool,
}

impl TerminalPose {
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
        if !self.turning && inside {
            self.turning = true;
            controller.pid.reset();
            changed = true;
        }

        let command = if self.turning {
            DifferentialTarget::Turn {
                heading: target.heading,
            }
        } else {
            DifferentialTarget::DriveToPoint {
                target: target.position,
                reverse,
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
            controller.pid.reset();
            controller.point_backing = None;
            controller.point_aligning = false;
            return (wheels, false, distance, true);
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
        assert!(changed && !ready && !terminal.turning);
    }
}
