//! Driver command shaping and pitch recovery for differential robots.

use std::time::Duration;

use crate::loops::{Feedback, Pid};

/// Limits changes in a normalized command without changing its steady-state value.
#[derive(Debug, Clone, Copy)]
pub struct SlewLimiter {
    value: f64,
    rise_per_second: f64,
    fall_per_second: f64,
}

impl SlewLimiter {
    pub const fn new(rise_per_second: f64, fall_per_second: f64) -> Self {
        Self {
            value: 0.0,
            rise_per_second,
            fall_per_second,
        }
    }

    pub fn update(&mut self, target: f64, dt: Duration) -> f64 {
        let target = target.clamp(-1.0, 1.0);
        let rate = if target.abs() > self.value.abs()
            && (self.value == 0.0 || target.signum() == self.value.signum())
        {
            self.rise_per_second
        } else {
            self.fall_per_second
        };
        let change = rate.max(0.0) * dt.as_secs_f64();
        self.value += (target - self.value).clamp(-change, change);
        self.value
    }

    pub const fn value(&self) -> f64 {
        self.value
    }

    pub fn reset(&mut self) {
        self.value = 0.0;
    }
}

/// Adds a bounded fore-aft drive correction when pitch exceeds a deadband.
#[derive(Debug, Clone, Copy)]
pub struct AntiTip {
    pid: Pid,
    threshold_radians: f64,
    polarity: f64,
    maximum_correction: f64,
}

impl AntiTip {
    pub const fn new(
        pid: Pid,
        threshold_radians: f64,
        polarity: f64,
        maximum_correction: f64,
    ) -> Self {
        Self {
            pid,
            threshold_radians,
            polarity,
            maximum_correction,
        }
    }

    pub fn correction(&mut self, pitch_radians: f64, dt: Duration) -> f64 {
        if !pitch_radians.is_finite() || pitch_radians.abs() < self.threshold_radians {
            self.pid = Pid::new(
                self.pid.kp(),
                self.pid.ki(),
                self.pid.kd(),
                self.pid.integration_range(),
            );
            return 0.0;
        }
        (self.polarity * self.pid.update(pitch_radians, 0.0, dt))
            .clamp(-self.maximum_correction, self.maximum_correction)
    }
}
