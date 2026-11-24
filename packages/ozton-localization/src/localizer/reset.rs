//! Coherent stationary collections for explicit distance resets.
use std::time::Duration;

use ozton_diagnostics::log_row;
use ozton_math::{
    geometry::{Pose, wrap_heading},
    localization::{
        observation::{MAX_RANGES, RangeObservation, RangeSensorConfig},
        range::{Constraint, Rejection, Surface, fit_translation},
    },
    uom::si::f64::{Angle, AngularVelocity, Length, Velocity},
};

use super::ranges::{RangeReport, constrain_sample};

/// Stationary evidence required before applying a geometric distance reset.
/// Scalar distances, velocities and angles are in SI units.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DsrConfig {
    pub maximum_linear_velocity: Velocity,
    pub maximum_angular_velocity: AngularVelocity,
    pub maximum_position_drift: Length,
    pub maximum_heading_drift: Angle,
    pub stationary_duration: Duration,
    pub collection_duration: Duration,
}

impl Default for DsrConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl DsrConfig {
    pub const DEFAULT: Self = Self {
        maximum_linear_velocity: ozton_math::qty!(0.025 m/s),
        maximum_angular_velocity: ozton_math::qty!(0.06 rad/s),
        maximum_position_drift: ozton_math::qty!(0.012 m),
        maximum_heading_drift: ozton_math::qty!(0.02 rad),
        stationary_duration: Duration::from_millis(120),
        collection_duration: Duration::from_millis(45),
    };

    pub const fn new() -> Self {
        Self::DEFAULT
    }

    #[must_use]
    pub const fn with_maximum_linear_velocity(mut self, value: Velocity) -> Self {
        self.maximum_linear_velocity = value;
        self
    }

    #[must_use]
    pub const fn with_maximum_angular_velocity(mut self, value: AngularVelocity) -> Self {
        self.maximum_angular_velocity = value;
        self
    }

    #[must_use]
    pub const fn with_maximum_position_drift(mut self, value: Length) -> Self {
        self.maximum_position_drift = value;
        self
    }

    #[must_use]
    pub const fn with_maximum_heading_drift(mut self, value: Angle) -> Self {
        self.maximum_heading_drift = value;
        self
    }

    #[must_use]
    pub const fn with_stationary_duration(mut self, value: Duration) -> Self {
        self.stationary_duration = value;
        self
    }

    #[must_use]
    pub const fn with_collection_duration(mut self, value: Duration) -> Self {
        self.collection_duration = value;
        self
    }

    pub fn valid(self) -> bool {
        [
            self.maximum_linear_velocity.value,
            self.maximum_angular_velocity.value,
            self.maximum_position_drift.value,
            self.maximum_heading_drift.value,
        ]
        .into_iter()
        .all(|value| value.is_finite() && value > 0.0)
            && !self.stationary_duration.is_zero()
            && !self.collection_duration.is_zero()
    }
}

pub(super) struct ResetWindow {
    anchor: Option<(Duration, Pose)>,
    samples: [Option<RangeObservation>; MAX_RANGES],
    collecting: Option<Duration>,
    minimum_axes: u8,
}

impl Default for ResetWindow {
    fn default() -> Self {
        Self::new(1)
    }
}

pub(super) enum ResetResult {
    Waiting,
    Applied(Pose, u8),
    Rejected,
}

pub(super) struct ResetTick {
    pub time: Duration,
    pub pose: Pose,
    pub forward_velocity: f64,
    pub lateral_velocity: f64,
    pub angular_velocity: f64,
}

impl ResetWindow {
    pub(super) fn new(minimum_axes: u8) -> Self {
        Self {
            anchor: None,
            samples: [None; MAX_RANGES],
            collecting: None,
            minimum_axes,
        }
    }

    pub(super) fn require_axes(&mut self, minimum_axes: u8) {
        if minimum_axes > self.minimum_axes {
            *self = Self::new(minimum_axes);
        }
    }

    pub(super) fn restart(&mut self) {
        *self = Self::new(self.minimum_axes);
    }
    /// Distance sensors publish independently. Collect fresh, nearby samples while stopped.
    pub(super) fn update(
        &mut self,
        tick: ResetTick,
        map: &[Surface],
        sensors: &[RangeSensorConfig],
        reports: &[Option<RangeReport>; MAX_RANGES],
        config: DsrConfig,
    ) -> ResetResult {
        let ResetTick {
            time: now,
            pose,
            forward_velocity,
            lateral_velocity,
            angular_velocity,
        } = tick;
        if ![forward_velocity, lateral_velocity, angular_velocity]
            .iter()
            .all(|v| v.is_finite())
            || forward_velocity * forward_velocity + lateral_velocity * lateral_velocity
                > config.maximum_linear_velocity.value * config.maximum_linear_velocity.value
            || angular_velocity.abs() > config.maximum_angular_velocity.value
        {
            log_row!(
                Trace,
                "DSR",
                "status=moving,v_forward_mps={forward_velocity:.4},v_lateral_mps={lateral_velocity:.4},yaw_radps={angular_velocity:.4}"
            );
            self.restart();
            return ResetResult::Waiting;
        }
        if self.anchor.is_some_and(|(_, anchor)| {
            pose.position.distance_squared(anchor.position)
                > config.maximum_position_drift.value * config.maximum_position_drift.value
                || wrap_heading(pose.heading - anchor.heading).value.abs()
                    > config.maximum_heading_drift.value
        }) {
            self.restart();
        }
        let (started, _) = *self.anchor.get_or_insert((now, pose));
        // Require the configured rest interval before collecting any packets.
        if now.saturating_sub(started) < config.stationary_duration {
            return ResetResult::Waiting;
        }
        let collection_start = *self.collecting.get_or_insert(now);
        for (stored, report) in self.samples.iter_mut().zip(reports) {
            if let Some(report) = report {
                match report.constraint {
                    Ok(_) => {
                        *stored = report
                            .observation
                            .filter(|v| v.timestamp >= collection_start);
                    }
                    Err(Rejection::Duplicate)
                        if report.observation.is_some_and(|sample| !sample.occluded) => {}
                    Err(_) => *stored = None,
                }
            }
        }
        if now.saturating_sub(collection_start) < config.collection_duration {
            return ResetResult::Waiting;
        }
        let mut constraints = [(Constraint::default(), 0.0); MAX_RANGES];
        let mut count = 0;
        for constraint in sensors
            .iter()
            .zip(self.samples)
            .filter_map(|(&sensor, sample)| {
                let sample = sample?;
                let age = now.checked_sub(sample.timestamp)?.as_secs_f64();
                let c = constrain_sample(pose, map, sensor, sample, age).ok()?;
                Some((
                    c,
                    sensor.constraint_sigma(c, (sample.distance - sensor.bias).value),
                ))
            })
        {
            constraints[count] = constraint;
            count += 1;
        }
        if count == 0 {
            log_row!(Trace, "DSR", "status=no_fresh_constraints");
            return ResetResult::Waiting;
        }
        if let Some((change, axes)) = fit_translation(&constraints[..count]) {
            if axes < self.minimum_axes {
                log_row!(
                    Trace,
                    "DSR",
                    "status=insufficient_axes,axes={axes},minimum_axes={},constraints={count}",
                    self.minimum_axes
                );
                return ResetResult::Waiting;
            }
            if change.is_finite() {
                log_row!(
                    Debug,
                    "DSR",
                    "status=applied,axes={axes},constraints={count},dx_m={:.5},dy_m={:.5},x_m={:.5},y_m={:.5},heading_rad={:.5}",
                    change.x,
                    change.y,
                    pose.position.x + change.x,
                    pose.position.y + change.y,
                    pose.heading.value
                );
                return ResetResult::Applied(
                    Pose {
                        position: pose.position + change,
                        ..pose
                    },
                    axes,
                );
            }
        }
        log_row!(Debug, "DSR", "status=rejected,constraints={count}");
        ResetResult::Rejected
    }
}
