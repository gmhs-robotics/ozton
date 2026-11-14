//! Position and velocity control for one actuator with measured soft limits.

use core::time::Duration;

use crate::loops::{Feedforward, MotorFeedforward, MotorFeedforwardSetpoint, Pid};

/// Builder-style settings for an axis in output radians and volts.
/// Call `build()` to validate settings before running the controller.
#[derive(Debug, Clone, Copy)]
pub struct AxisConfig {
    pub pid: Pid,
    pub feedforward: MotorFeedforward,
    pub derivative_filter: Duration,
    pub maximum_voltage: f64,
    pub position_limits: Option<(f64, f64)>,
}

impl Default for AxisConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl AxisConfig {
    /// Starts a stationary axis with zero gains, no limits, and a 12 V cap.
    /// Positions and velocities use output radians and radians per second.
    pub fn new() -> Self {
        Self {
            pid: Pid::new(0.0, 0.0, 0.0, None),
            feedforward: MotorFeedforward::new(0.0, 0.0, 0.0),
            derivative_filter: Duration::ZERO,
            maximum_voltage: 12.0,
            position_limits: None,
        }
    }

    #[must_use]
    pub fn with_pid(mut self, pid: Pid) -> Self {
        self.pid = pid;
        self
    }

    #[must_use]
    pub fn with_feedforward(mut self, feedforward: MotorFeedforward) -> Self {
        self.feedforward = feedforward;
        self
    }

    #[must_use]
    pub fn with_derivative_filter(mut self, filter: Duration) -> Self {
        self.derivative_filter = filter;
        self
    }

    #[must_use]
    pub fn with_maximum_voltage(mut self, voltage: f64) -> Self {
        self.maximum_voltage = voltage;
        self
    }

    #[must_use]
    pub fn with_position_limits(mut self, bottom: f64, top: f64) -> Self {
        self.position_limits = Some((bottom, top));
        self
    }

    /// Validates settings before creating the controller.
    pub fn build(self) -> Option<AxisController> {
        AxisController::new(self)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AxisSample {
    pub position: f64,
    pub velocity: f64,
}

#[derive(Debug, Clone, Copy)]
pub enum Target {
    Position {
        position: f64,
        velocity: f64,
        acceleration: f64,
    },
    Velocity {
        velocity: f64,
        acceleration: f64,
    },
}

#[derive(Debug, Clone, Copy)]
pub struct AxisController {
    config: AxisConfig,
    position_tolerance: f64,
    maximum_gap: Option<Duration>,
    position_mode: Option<bool>,
}

impl AxisController {
    pub fn new(mut config: AxisConfig) -> Option<Self> {
        let (kp, ki, kd) = config.pid.gains();
        let (ks, kv, ka) = config.feedforward.constants();
        if ![kp, ki, kd, ks, kv, ka]
            .iter()
            .all(|value| value.is_finite())
            || config
                .pid
                .integration_range()
                .is_some_and(|range| !range.is_finite() || range <= 0.0)
            || !config.maximum_voltage.is_finite()
            || config.maximum_voltage <= 0.0
            || config
                .position_limits
                .is_some_and(|(min, max)| !min.is_finite() || !max.is_finite() || min >= max)
        {
            return None;
        }
        config.pid.set_output_limit(Some(config.maximum_voltage));
        config.pid.reset();
        Some(Self {
            config,
            position_tolerance: 0.0,
            maximum_gap: None,
            position_mode: None,
        })
    }

    /// Adds a position deadband in output radians. Invalid values fail closed on update.
    /// Stationary targets inside the deadband produce zero volts; callers choose braking.
    #[must_use]
    pub fn with_position_tolerance(mut self, tolerance: f64) -> Self {
        self.position_tolerance = tolerance;
        self
    }

    /// Rejects stale control ticks; omit only when timing is managed externally.
    #[must_use]
    pub fn with_maximum_gap(mut self, gap: Duration) -> Self {
        self.maximum_gap = Some(gap);
        self
    }

    pub fn reset(&mut self) {
        self.config.pid.reset();
        self.position_mode = None;
    }

    pub fn update(&mut self, sample: AxisSample, target: Target, dt: Duration) -> Option<f64> {
        let valid_target = match target {
            Target::Position {
                position,
                velocity,
                acceleration,
            } => [position, velocity, acceleration]
                .iter()
                .all(|v| v.is_finite()),
            Target::Velocity {
                velocity,
                acceleration,
            } => [velocity, acceleration].iter().all(|v| v.is_finite()),
        };
        if !valid_target || !self.position_tolerance.is_finite() || self.position_tolerance < 0.0 {
            self.reset();
            return None;
        }
        let (measurement, setpoint, velocity, acceleration, position_mode) = match target {
            Target::Position {
                position,
                velocity,
                acceleration,
            } => {
                let setpoint = self
                    .config
                    .position_limits
                    .map_or(position, |(min, max)| position.clamp(min, max));
                (sample.position, setpoint, velocity, acceleration, true)
            }
            Target::Velocity {
                velocity,
                acceleration,
            } => (sample.velocity, velocity, velocity, acceleration, false),
        };
        if dt.is_zero()
            || self.maximum_gap.is_some_and(|gap| dt > gap)
            || ![
                sample.position,
                sample.velocity,
                setpoint,
                velocity,
                acceleration,
            ]
            .iter()
            .all(|value| value.is_finite())
        {
            self.reset();
            return None;
        }
        if self.position_mode != Some(position_mode) {
            self.config.pid.reset();
            self.position_mode = Some(position_mode);
        }
        if self.position_tolerance > 0.0
            && position_mode
            && (setpoint - measurement).abs() <= self.position_tolerance
            && velocity == 0.0
            && acceleration == 0.0
        {
            self.config.pid.reset();
            return Some(0.0);
        }
        let feedforward = self.config.feedforward.update(
            MotorFeedforwardSetpoint {
                velocity,
                acceleration,
            },
            dt,
        );
        if !feedforward.is_finite() {
            self.reset();
            return None;
        }
        let output = self.config.pid.update_filtered(
            measurement,
            setpoint,
            dt,
            self.config.derivative_filter,
            feedforward,
        );
        if !output.is_finite() {
            self.reset();
            return None;
        }
        let limited = self.config.position_limits.is_some_and(|(min, max)| {
            (sample.position <= min && output < 0.0) || (sample.position >= max && output > 0.0)
        });
        if limited {
            self.config.pid.reset();
            Some(0.0)
        } else {
            Some(output)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TICK: Duration = Duration::from_millis(10);
    const SAMPLE: AxisSample = AxisSample {
        position: 0.5,
        velocity: 0.0,
    };

    fn target(position: f64) -> Target {
        Target::Position {
            position,
            velocity: 0.0,
            acceleration: 0.0,
        }
    }

    fn controller() -> AxisController {
        AxisConfig::new()
            .with_pid(Pid::new(10.0, 0.0, 0.0, None))
            .with_maximum_voltage(4.0)
            .with_position_limits(0.0, 1.0)
            .build()
            .unwrap()
            .with_position_tolerance(0.01)
            .with_maximum_gap(Duration::from_millis(100))
    }

    #[test]
    fn builder_limits_voltage_travel_and_timing_and_recovers_after_faults() {
        let mut axis = controller();
        assert_eq!(axis.update(SAMPLE, target(100.0), TICK), Some(4.0));
        assert_eq!(axis.update(SAMPLE, target(-100.0), TICK), Some(-4.0));
        assert_eq!(axis.update(SAMPLE, target(0.505), TICK), Some(0.0));
        for dt in [Duration::ZERO, Duration::from_millis(101)] {
            assert!(axis.update(SAMPLE, target(1.0), dt).is_none());
        }
        assert_eq!(axis.update(SAMPLE, target(1.0), TICK), Some(4.0));
        // An axis outside its measured limits must still be allowed to return.
        assert_eq!(
            axis.update(
                AxisSample {
                    position: 1.5,
                    velocity: 0.0
                },
                target(1.0),
                TICK
            ),
            Some(-4.0)
        );
    }

    #[test]
    fn invalid_raw_targets_cannot_become_valid_by_clamping() {
        let mut axis = controller();
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(axis.update(SAMPLE, target(invalid), TICK).is_none());
            assert!(
                axis.update(
                    AxisSample {
                        position: invalid,
                        velocity: 0.0
                    },
                    target(1.0),
                    TICK
                )
                .is_none()
            );
        }
        assert!(
            AxisConfig::new()
                .with_maximum_voltage(f64::NAN)
                .build()
                .is_none()
        );
        assert!(
            AxisConfig::new()
                .with_position_limits(1.0, 0.0)
                .build()
                .is_none()
        );
        assert!(
            controller()
                .with_position_tolerance(-1.0)
                .update(SAMPLE, target(0.5), TICK)
                .is_none()
        );
    }
}
