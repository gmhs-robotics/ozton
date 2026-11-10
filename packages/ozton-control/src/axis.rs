//! Position and velocity control for one actuator with measured soft limits.

use core::time::Duration;

use crate::loops::{Feedforward, MotorFeedforward, MotorFeedforwardSetpoint, Pid};

#[derive(Debug, Clone, Copy)]
pub struct AxisConfig {
    pub pid: Pid,
    pub feedforward: MotorFeedforward,
    pub derivative_filter: Duration,
    pub maximum_voltage: f64,
    pub position_limits: Option<(f64, f64)>,
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
            position_mode: None,
        })
    }

    pub fn reset(&mut self) {
        self.config.pid.reset();
        self.position_mode = None;
    }

    pub fn update(&mut self, sample: AxisSample, target: Target, dt: Duration) -> Option<f64> {
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
