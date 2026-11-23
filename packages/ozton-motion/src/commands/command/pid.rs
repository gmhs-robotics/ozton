//! Per-command forward and yaw feedback in SI units.

use ozton_math::control::Pid;
use ozton_runtime::log_row;

use super::*;

const RATE_FILTER: Duration = Duration::from_millis(55);

pub(in crate::commands) const fn angular_pid() -> Pid {
    Pid::new(5.4, 1.1, 0.24, Some(0.24))
}

pub(super) struct MotionPid {
    forward: Pid,
    angular: Pid,
}

impl Default for MotionPid {
    fn default() -> Self {
        Self {
            // The Match trace spent 5.4 s approaching the last 0.20 m. Use a
            // stronger approach, with velocity damping and integral near the goal.
            forward: Pid::new(4.2, 0.9, 0.18, Some(0.08)),
            // The previous 3.0 P turn stopped near the 2-degree threshold.
            angular: angular_pid(),
        }
    }
}

impl MotionPid {
    pub(super) fn reset(&mut self) {
        self.forward.reset();
        self.angular.reset();
    }

    pub(super) fn forward(
        &mut self,
        error: f64,
        measured_rate: f64,
        limit: f64,
        dt: Duration,
    ) -> f64 {
        if limit <= 0.0 {
            self.forward.reset();
            return 0.0;
        }
        self.forward.set_output_limit(Some(limit));
        let output = self
            .forward
            .update_with_rate(error, -measured_rate, dt, RATE_FILTER, 0.0);
        log_row!(
            Trace,
            "MOTION_PID",
            "axis=forward,error_m={error:.5},rate_mps={measured_rate:.5},limit_mps={limit:.5},output_mps={output:.5}"
        );
        output
    }

    pub(super) fn yaw(
        &mut self,
        error: f64,
        measured_rate: f64,
        feedforward: f64,
        limit: f64,
        dt: Duration,
    ) -> f64 {
        self.angular.set_output_limit(Some(limit));
        let output = feedforward
            + self.angular.update_with_rate(
                error,
                feedforward - measured_rate,
                dt,
                RATE_FILTER,
                0.0,
            );
        log_row!(
            Trace,
            "MOTION_PID",
            "axis=yaw,error_rad={error:.5},rate_radps={measured_rate:.5},feedforward_radps={feedforward:.5},correction_limit_radps={limit:.5},output_radps={output:.5}"
        );
        output
    }
}
