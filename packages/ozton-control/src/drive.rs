//! Driver command shaping and pitch recovery for differential robots.

use std::time::Duration;

use ozton_math::uom::si::f64::{Angle, Frequency};

use crate::loops::{Feedback, Pid};

/// Limits changes in a normalized command without changing its steady-state value.
#[derive(Debug, Clone, Copy)]
pub struct SlewLimiter {
    value: f64,
    rise_per_second: Frequency,
    fall_per_second: Frequency,
}

impl SlewLimiter {
    pub const fn new(rise_per_second: Frequency, fall_per_second: Frequency) -> Self {
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
        let change = rate.value.max(0.0) * dt.as_secs_f64();
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
    pub const fn new(pid: Pid, threshold: Angle, polarity: f64, maximum_correction: f64) -> Self {
        Self {
            pid,
            threshold_radians: threshold.value,
            polarity,
            maximum_correction,
        }
    }

    pub fn reset(&mut self) {
        self.pid.reset();
    }

    pub fn correction(&mut self, pitch: Angle, dt: Duration) -> f64 {
        let pitch_radians = pitch.value;
        if !pitch_radians.is_finite()
            || !self.threshold_radians.is_finite()
            || self.threshold_radians < 0.0
            || self.polarity.abs() != 1.0
            || !(0.0..=1.0).contains(&self.maximum_correction)
            || pitch_radians.abs() <= self.threshold_radians
        {
            self.reset();
            return 0.0;
        }
        let excess_tilt = pitch_radians.signum() * (pitch_radians.abs() - self.threshold_radians);
        (self.polarity * self.pid.update(excess_tilt, 0.0, dt))
            .clamp(-self.maximum_correction, self.maximum_correction)
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn anti_tip_is_neutral_near_level_and_recovers_in_both_directions() {
        let mut control = AntiTip::new(
            Pid::new(1.35, 0.0, 0.03, None),
            ozton_math::angle!(0.16 rad),
            1.0,
            0.35,
        );
        let dt = Duration::from_millis(10);
        for tilt in [-0.1, 0.0, 0.1] {
            assert_eq!(control.correction(ozton_math::angle!(tilt, rad), dt), 0.0);
        }
        for tilt in [-0.3, 0.3] {
            let correction = control.correction(ozton_math::angle!(tilt, rad), dt);
            assert!(correction * tilt < 0.0);
            assert!(correction.abs() <= 0.35);
            assert_eq!(control.correction(ozton_math::angle!(0.0 rad), dt), 0.0);
        }
    }
}
