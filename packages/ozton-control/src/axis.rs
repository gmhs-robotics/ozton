//! Position and velocity control for one actuator with measured soft limits.

use core::time::Duration;

pub use ozton_math::control::axis::{
    Acceleration, AxisDimension, Linear, Position, Rotary, Velocity,
};
use ozton_math::uom::si::{electric_potential::volt, f64::ElectricPotential};

use crate::loops::{Feedforward, MotorFeedforward, MotorFeedforwardSetpoint, Pid};

/// Builder-style settings for an axis with typed position limits and voltage.
/// Call `build()` to validate settings before running the controller.
#[derive(Debug, Clone, Copy)]
pub struct AxisConfig<D: AxisDimension = Rotary> {
    pub pid: Pid,
    pub feedforward: MotorFeedforward<D>,
    pub derivative_filter: Duration,
    pub maximum_voltage: ElectricPotential,
    pub position_limits: Option<(Position<D>, Position<D>)>,
}

impl Default for AxisConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl AxisConfig {
    /// Starts a rotary axis with zero gains, no limits, and a 12 V cap.
    pub fn new() -> Self {
        Self::with_defaults()
    }
}

impl AxisConfig<Linear> {
    /// Starts a linear axis with zero gains, no limits, and a 12 V cap.
    pub fn linear() -> Self {
        Self::with_defaults()
    }
}

impl<D: AxisDimension> AxisConfig<D> {
    fn with_defaults() -> Self {
        Self {
            pid: Pid::new(0.0, 0.0, 0.0, None),
            feedforward: MotorFeedforward::new(ElectricPotential::new::<volt>(0.0), 0.0, 0.0),
            derivative_filter: Duration::ZERO,
            maximum_voltage: ElectricPotential::new::<volt>(12.0),
            position_limits: None,
        }
    }

    #[must_use]
    pub fn with_pid(mut self, pid: Pid) -> Self {
        self.pid = pid;
        self
    }

    #[must_use]
    pub fn with_feedforward(mut self, feedforward: MotorFeedforward<D>) -> Self {
        self.feedforward = feedforward;
        self
    }

    #[must_use]
    pub fn with_derivative_filter(mut self, filter: Duration) -> Self {
        self.derivative_filter = filter;
        self
    }

    #[must_use]
    pub fn with_maximum_voltage(mut self, voltage: ElectricPotential) -> Self {
        self.maximum_voltage = voltage;
        self
    }

    #[must_use]
    pub fn with_position_limits(mut self, bottom: Position<D>, top: Position<D>) -> Self {
        self.position_limits = Some((bottom, top));
        self
    }

    /// Validates settings before creating the controller.
    pub fn build(self) -> Option<AxisController<D>> {
        AxisController::new(self)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AxisSample<D: AxisDimension = Rotary> {
    pub position: Position<D>,
    pub velocity: Velocity<D>,
}

#[derive(Debug, Clone, Copy)]
pub enum Target<D: AxisDimension = Rotary> {
    Position {
        position: Position<D>,
        velocity: Velocity<D>,
        acceleration: Acceleration<D>,
    },
    Velocity {
        velocity: Velocity<D>,
        acceleration: Acceleration<D>,
    },
}

#[derive(Debug, Clone, Copy)]
pub struct AxisController<D: AxisDimension = Rotary> {
    config: AxisConfig<D>,
    position_tolerance: f64,
    maximum_gap: Option<Duration>,
    position_mode: Option<bool>,
}

impl<D: AxisDimension> AxisController<D> {
    pub fn new(mut config: AxisConfig<D>) -> Option<Self> {
        let (kp, ki, kd) = config.pid.gains();
        let (ks, kv, ka) = config.feedforward.constants();
        if ![kp, ki, kd, ks.value, kv, ka]
            .iter()
            .all(|value| value.is_finite())
            || config
                .pid
                .integration_range()
                .is_some_and(|range| !range.is_finite() || range <= 0.0)
            || !config.maximum_voltage.value.is_finite()
            || config.maximum_voltage.value <= 0.0
            || config.position_limits.is_some_and(|(min, max)| {
                !min.value.is_finite() || !max.value.is_finite() || min >= max
            })
        {
            return None;
        }
        config
            .pid
            .set_output_limit(Some(config.maximum_voltage.value));
        config.pid.reset();
        Some(Self {
            config,
            position_tolerance: 0.0,
            maximum_gap: None,
            position_mode: None,
        })
    }

    /// Adds a position deadband. Invalid values fail closed on update.
    /// Stationary targets inside the deadband produce zero volts; callers choose braking.
    #[must_use]
    pub fn with_position_tolerance(mut self, tolerance: Position<D>) -> Self {
        self.position_tolerance = tolerance.value;
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

    pub fn update(
        &mut self,
        sample: AxisSample<D>,
        target: Target<D>,
        dt: Duration,
    ) -> Option<ElectricPotential> {
        let valid_target = match target {
            Target::Position {
                position,
                velocity,
                acceleration,
            } => [position.value, velocity.value, acceleration.value]
                .iter()
                .all(|v| v.is_finite()),
            Target::Velocity {
                velocity,
                acceleration,
            } => [velocity.value, acceleration.value]
                .iter()
                .all(|v| v.is_finite()),
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
                    .map_or(position.value, |(min, max)| {
                        position.value.clamp(min.value, max.value)
                    });
                (
                    sample.position.value,
                    setpoint,
                    velocity,
                    acceleration,
                    true,
                )
            }
            Target::Velocity {
                velocity,
                acceleration,
            } => (
                sample.velocity.value,
                velocity.value,
                velocity,
                acceleration,
                false,
            ),
        };
        if dt.is_zero()
            || self.maximum_gap.is_some_and(|gap| dt > gap)
            || ![
                sample.position.value,
                sample.velocity.value,
                setpoint,
                velocity.value,
                acceleration.value,
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
            && velocity.value == 0.0
            && acceleration.value == 0.0
        {
            self.config.pid.reset();
            return Some(ElectricPotential::new::<volt>(0.0));
        }
        let feedforward = self.config.feedforward.update(
            MotorFeedforwardSetpoint {
                velocity,
                acceleration,
            },
            dt,
        );
        if !feedforward.value.is_finite() {
            self.reset();
            return None;
        }
        let output = self.config.pid.update_filtered(
            measurement,
            setpoint,
            dt,
            self.config.derivative_filter,
            feedforward.value,
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
            Some(ElectricPotential::new::<volt>(0.0))
        } else {
            Some(ElectricPotential::new::<volt>(output))
        }
    }
}

#[cfg(test)]
mod tests {
    use ozton_math::uom::{
        ConstZero,
        si::{
            angle::radian,
            angular_acceleration::radian_per_second_squared,
            angular_velocity::radian_per_second,
            f64::{Angle, AngularAcceleration, AngularVelocity},
        },
    };

    use super::*;

    const TICK: Duration = Duration::from_millis(10);
    const SAMPLE: AxisSample = AxisSample {
        position: Angle {
            value: 0.5,
            ..Angle::ZERO
        },
        velocity: AngularVelocity::ZERO,
    };

    fn target(position: f64) -> Target {
        Target::Position {
            position: Angle::new::<radian>(position),
            velocity: AngularVelocity::new::<radian_per_second>(0.0),
            acceleration: AngularAcceleration::new::<radian_per_second_squared>(0.0),
        }
    }

    fn controller() -> AxisController {
        AxisConfig::new()
            .with_pid(Pid::new(10.0, 0.0, 0.0, None))
            .with_maximum_voltage(ElectricPotential::new::<volt>(4.0))
            .with_position_limits(Angle::new::<radian>(0.0), Angle::new::<radian>(1.0))
            .build()
            .unwrap()
            .with_position_tolerance(Angle::new::<radian>(0.01))
            .with_maximum_gap(Duration::from_millis(100))
    }

    #[test]
    fn builder_limits_voltage_travel_and_timing_and_recovers_after_faults() {
        let mut axis = controller();
        assert_eq!(
            axis.update(SAMPLE, target(100.0), TICK),
            Some(ElectricPotential::new::<volt>(4.0))
        );
        assert_eq!(
            axis.update(SAMPLE, target(-100.0), TICK),
            Some(ElectricPotential::new::<volt>(-4.0))
        );
        assert_eq!(
            axis.update(SAMPLE, target(0.505), TICK),
            Some(ElectricPotential::new::<volt>(0.0))
        );
        for dt in [Duration::ZERO, Duration::from_millis(101)] {
            assert!(axis.update(SAMPLE, target(1.0), dt).is_none());
        }
        assert_eq!(
            axis.update(SAMPLE, target(1.0), TICK),
            Some(ElectricPotential::new::<volt>(4.0))
        );
        // An axis outside its measured limits must still be allowed to return.
        assert_eq!(
            axis.update(
                AxisSample {
                    position: Angle::new::<radian>(1.5),
                    velocity: AngularVelocity::new::<radian_per_second>(0.0)
                },
                target(1.0),
                TICK
            ),
            Some(ElectricPotential::new::<volt>(-4.0))
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
                        position: Angle::new::<radian>(invalid),
                        velocity: AngularVelocity::new::<radian_per_second>(0.0)
                    },
                    target(1.0),
                    TICK
                )
                .is_none()
            );
        }
        assert!(
            AxisConfig::new()
                .with_maximum_voltage(ElectricPotential::new::<volt>(f64::NAN))
                .build()
                .is_none()
        );
        assert!(
            AxisConfig::new()
                .with_position_limits(Angle::new::<radian>(1.0), Angle::new::<radian>(0.0))
                .build()
                .is_none()
        );
        assert!(
            controller()
                .with_position_tolerance(Angle::new::<radian>(-1.0))
                .update(SAMPLE, target(0.5), TICK)
                .is_none()
        );
    }
}
