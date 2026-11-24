//! Per-command forward and yaw feedback in SI units.

use std::time::Duration;

use ozton_math::control::Pid;
#[cfg(feature = "logging")]
use ozton_runtime::log_row;

use super::DriveTuning;

const RATE_FILTER: Duration = Duration::from_millis(55);

pub struct MotionPid {
    forward: Pid,
    angular: Pid,
    forward_limit: f64,
    angular_limit: f64,
}

impl Default for MotionPid {
    fn default() -> Self {
        Self::new(DriveTuning::STANDARD)
    }
}

impl MotionPid {
    pub fn new(tuning: DriveTuning) -> Self {
        let mut pid = Self {
            forward: tuning.linear,
            angular: tuning.angular,
            forward_limit: tuning.linear.output_limit().unwrap_or(f64::INFINITY),
            angular_limit: tuning.angular.output_limit().unwrap_or(f64::INFINITY),
        };
        pid.reset();
        pid
    }

    pub fn reset(&mut self) {
        self.forward.reset();
        self.angular.reset();
    }

    pub fn forward(&mut self, error: f64, measured_rate: f64, limit: f64, dt: Duration) -> f64 {
        self.forward
            .set_output_limit(Some(limit.max(0.0).min(self.forward_limit)));
        let output = self
            .forward
            .update_with_rate(error, -measured_rate, dt, RATE_FILTER, 0.0);
        #[cfg(feature = "logging")]
        log_row!(
            Trace,
            "MOTION_PID",
            "axis=forward,error_m={error:.5},rate_mps={measured_rate:.5},limit_mps={limit:.5},output_mps={output:.5}"
        );
        output
    }

    pub fn yaw(
        &mut self,
        error: f64,
        measured_rate: f64,
        feedforward: f64,
        limit: f64,
        dt: Duration,
    ) -> f64 {
        self.angular
            .set_output_limit(Some(limit.min(self.angular_limit)));
        let output = feedforward
            + self.angular.update_with_rate(
                error,
                feedforward - measured_rate,
                dt,
                RATE_FILTER,
                0.0,
            );
        #[cfg(feature = "logging")]
        log_row!(
            Trace,
            "MOTION_PID",
            "axis=yaw,error_rad={error:.5},rate_radps={measured_rate:.5},feedforward_radps={feedforward:.5},correction_limit_radps={limit:.5},output_radps={output:.5}"
        );
        output
    }
}
