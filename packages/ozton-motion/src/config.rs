//! Robot-specific physical motion configuration. All scalar dynamics use SI units.
use std::time::Duration;

use ozton_math::uom::si::f64::{
    Acceleration, Angle, AngularAcceleration, AngularVelocity, Length, Velocity,
};

use crate::basic::differential::{DriveOptions, DriveTuning};

/// Kinematic limits and optional breakaway compensation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionLimits {
    pub wheel_speed: Velocity,
    /// Optional independently characterized wheel acceleration limit.
    pub wheel_acceleration: Option<Acceleration>,
    /// Breakaway compensation, applied only to active forward feedback near rest.
    pub minimum_linear_speed: Velocity,
    /// Breakaway compensation, applied only to active yaw feedback near rest.
    pub minimum_angular_speed: AngularVelocity,
    pub acceleration: Acceleration,
    pub deceleration: Acceleration,
    /// Measured actuator delay used by endpoint braking feedback.
    pub brake_latency: Duration,
    /// Measured forward braking, or the profile deceleration if unspecified.
    pub forward_braking: Option<Acceleration>,
    /// Measured yaw braking, or the profile angular deceleration if unspecified.
    pub yaw_braking: Option<AngularAcceleration>,
    pub lateral_acceleration: Acceleration,
    pub angular_speed: AngularVelocity,
    /// rad/s²
    pub angular_acceleration: AngularAcceleration,
    /// rad/s²
    pub angular_deceleration: AngularAcceleration,
}

impl Default for MotionLimits {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl MotionLimits {
    pub const DEFAULT: Self = Self {
        wheel_speed: ozton_math::units::qty!(0.50 m/s),
        wheel_acceleration: None,
        minimum_linear_speed: ozton_math::qty!(0 m/s),
        minimum_angular_speed: ozton_math::qty!(0 rad/s),
        acceleration: ozton_math::units::qty!(0.8 m/s^2),
        deceleration: ozton_math::units::qty!(2.0 m/s^2),
        brake_latency: Duration::from_millis(100),
        forward_braking: None,
        yaw_braking: None,
        lateral_acceleration: ozton_math::units::qty!(0.8 m/s^2),
        angular_speed: ozton_math::units::qty!(5.0 rad/s),
        angular_acceleration: ozton_math::qty!(8.0 rad/s^2),
        angular_deceleration: ozton_math::qty!(8.0 rad/s^2),
    };

    pub const fn new() -> Self {
        Self::DEFAULT
    }

    #[must_use]
    pub const fn with_wheel_speed(mut self, value: Velocity) -> Self {
        self.wheel_speed = value;
        self
    }

    #[must_use]
    pub const fn with_wheel_acceleration(mut self, value: Acceleration) -> Self {
        self.wheel_acceleration = Some(value);
        self
    }

    #[must_use]
    pub const fn with_minimum_linear_speed(mut self, value: Velocity) -> Self {
        self.minimum_linear_speed = value;
        self
    }

    #[must_use]
    pub const fn with_minimum_angular_speed(mut self, value: AngularVelocity) -> Self {
        self.minimum_angular_speed = value;
        self
    }

    #[must_use]
    pub const fn with_acceleration(mut self, value: Acceleration) -> Self {
        self.acceleration = value;
        self
    }

    #[must_use]
    pub const fn with_deceleration(mut self, value: Acceleration) -> Self {
        self.deceleration = value;
        self
    }

    #[must_use]
    pub const fn with_brake_latency(mut self, value: Duration) -> Self {
        self.brake_latency = value;
        self
    }

    #[must_use]
    pub const fn with_forward_braking(mut self, value: Acceleration) -> Self {
        self.forward_braking = Some(value);
        self
    }

    #[must_use]
    pub const fn with_yaw_braking(mut self, value: AngularAcceleration) -> Self {
        self.yaw_braking = Some(value);
        self
    }

    #[must_use]
    pub const fn with_lateral_acceleration(mut self, value: Acceleration) -> Self {
        self.lateral_acceleration = value;
        self
    }

    #[must_use]
    pub const fn with_angular_speed(mut self, value: AngularVelocity) -> Self {
        self.angular_speed = value;
        self
    }

    #[must_use]
    pub const fn with_angular_acceleration(mut self, value: AngularAcceleration) -> Self {
        self.angular_acceleration = value;
        self
    }

    #[must_use]
    pub const fn with_angular_deceleration(mut self, value: AngularAcceleration) -> Self {
        self.angular_deceleration = value;
        self
    }
}

/// Optional feedback rate filtering.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RateFilters {
    pub linear_rate_filter: Duration,
    pub angular_rate_filter: Duration,
}

impl Default for RateFilters {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl RateFilters {
    pub const DEFAULT: Self = Self {
        linear_rate_filter: Duration::from_millis(55),
        angular_rate_filter: Duration::from_millis(55),
    };

    pub const fn new() -> Self {
        Self::DEFAULT
    }

    #[must_use]
    pub const fn with_linear_rate_filter(mut self, value: Duration) -> Self {
        self.linear_rate_filter = value;
        self
    }

    #[must_use]
    pub const fn with_angular_rate_filter(mut self, value: Duration) -> Self {
        self.angular_rate_filter = value;
        self
    }
}

/// Arrival tolerances and continuous settling duration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SettlingConfig {
    pub position_tolerance: Length,
    pub heading_tolerance: Angle,
    pub velocity_tolerance: Velocity,
    pub angular_velocity_tolerance: AngularVelocity,
    pub settle: Duration,
}

impl Default for SettlingConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl SettlingConfig {
    pub const DEFAULT: Self = Self {
        position_tolerance: ozton_math::qty!(0.025 m),
        heading_tolerance: ozton_math::qty!(0.06 rad),
        velocity_tolerance: ozton_math::units::qty!(0.04 m/s),
        angular_velocity_tolerance: ozton_math::units::qty!(0.12 rad/s),
        settle: Duration::from_millis(120),
    };

    pub const fn new() -> Self {
        Self::DEFAULT
    }

    #[must_use]
    pub const fn with_position_tolerance(mut self, value: Length) -> Self {
        self.position_tolerance = value;
        self
    }

    #[must_use]
    pub const fn with_heading_tolerance(mut self, value: Angle) -> Self {
        self.heading_tolerance = value;
        self
    }

    #[must_use]
    pub const fn with_velocity_tolerance(mut self, value: Velocity) -> Self {
        self.velocity_tolerance = value;
        self
    }

    #[must_use]
    pub const fn with_angular_velocity_tolerance(mut self, value: AngularVelocity) -> Self {
        self.angular_velocity_tolerance = value;
        self
    }

    #[must_use]
    pub const fn with_settle(mut self, value: Duration) -> Self {
        self.settle = value;
        self
    }
}

/// Control scheduling and progress deadlines.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionTiming {
    pub period: Duration,
    pub maximum_gap: Duration,
    /// Elapsed time without progress before issuing a diagnostic warning.
    pub progress_timeout: Duration,
    /// Expected stage duration; exceeding it issues a diagnostic warning.
    pub stage_timeout: Duration,
}

impl Default for MotionTiming {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl MotionTiming {
    pub const DEFAULT: Self = Self {
        period: Duration::from_millis(10),
        maximum_gap: Duration::from_millis(100),
        progress_timeout: Duration::from_millis(2500),
        stage_timeout: Duration::from_secs(8),
    };

    pub const fn new() -> Self {
        Self::DEFAULT
    }

    #[must_use]
    pub const fn with_period(mut self, value: Duration) -> Self {
        self.period = value;
        self
    }

    #[must_use]
    pub const fn with_maximum_gap(mut self, value: Duration) -> Self {
        self.maximum_gap = value;
        self
    }

    #[must_use]
    pub const fn with_progress_timeout(mut self, value: Duration) -> Self {
        self.progress_timeout = value;
        self
    }

    #[must_use]
    pub const fn with_stage_timeout(mut self, value: Duration) -> Self {
        self.stage_timeout = value;
        self
    }
}

/// Geometric clearance used to validate authored motions before execution.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClearanceConfig {
    pub clearance: Length,
}

impl Default for ClearanceConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl ClearanceConfig {
    pub const DEFAULT: Self = Self {
        clearance: ozton_math::qty!(0.5 in),
    };

    pub const fn new() -> Self {
        Self::DEFAULT
    }

    #[must_use]
    pub const fn with_clearance(mut self, value: Length) -> Self {
        self.clearance = value;
        self
    }
}

/// Trajectory follower parameters.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum PathGuidance {
    /// Spatial tangent feedback with measured yaw damping.
    #[default]
    Tangent,
    /// Geometric pursuit of an arc-length lookahead point.
    PurePursuit,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathConfig {
    pub guidance: PathGuidance,
    pub lookahead: Length,
    /// Cross-track error threshold for a diagnostic warning, not an abort.
    pub maximum_cross_track_error: Length,
}

impl Default for PathConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl PathConfig {
    pub const DEFAULT: Self = Self {
        guidance: PathGuidance::Tangent,
        lookahead: ozton_math::qty!(0.16 m),
        maximum_cross_track_error: ozton_math::qty!(0.20 m),
    };

    pub const fn new() -> Self {
        Self::DEFAULT
    }

    #[must_use]
    pub const fn with_guidance(mut self, guidance: PathGuidance) -> Self {
        self.guidance = guidance;
        self
    }

    #[must_use]
    pub const fn with_lookahead(mut self, value: Length) -> Self {
        self.lookahead = value;
        self
    }

    #[must_use]
    pub const fn with_maximum_cross_track_error(mut self, value: Length) -> Self {
        self.maximum_cross_track_error = value;
        self
    }
}

/// Point approach and terminal correction behavior.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ApproachConfig {
    /// Fraction of remaining distance projected behind the target pose.
    pub boomerang_lead: f64,
    pub reversal_heading: Angle,
    pub reversal_distance_factor: f64,
    pub reversal_yaw_rate: AngularVelocity,
    pub arc_correction_limit: AngularVelocity,
    pub terminal_arc_correction_limit: AngularVelocity,
}

impl Default for ApproachConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl ApproachConfig {
    pub const DEFAULT: Self = Self {
        boomerang_lead: 0.6,
        reversal_heading: ozton_math::qty!(125 deg),
        reversal_distance_factor: 4.0,
        reversal_yaw_rate: ozton_math::units::qty!(0.5 rad/s),
        arc_correction_limit: ozton_math::units::qty!(0.35 rad/s),
        terminal_arc_correction_limit: ozton_math::units::qty!(0.5 rad/s),
    };

    pub const fn new() -> Self {
        Self::DEFAULT
    }

    #[must_use]
    pub const fn with_boomerang_lead(mut self, value: f64) -> Self {
        self.boomerang_lead = value;
        self
    }

    #[must_use]
    pub const fn with_reversal_heading(mut self, value: Angle) -> Self {
        self.reversal_heading = value;
        self
    }

    #[must_use]
    pub const fn with_reversal_distance_factor(mut self, value: f64) -> Self {
        self.reversal_distance_factor = value;
        self
    }

    #[must_use]
    pub const fn with_reversal_yaw_rate(mut self, value: AngularVelocity) -> Self {
        self.reversal_yaw_rate = value;
        self
    }

    #[must_use]
    pub const fn with_arc_correction_limit(mut self, value: AngularVelocity) -> Self {
        self.arc_correction_limit = value;
        self
    }

    #[must_use]
    pub const fn with_terminal_arc_correction_limit(mut self, value: AngularVelocity) -> Self {
        self.terminal_arc_correction_limit = value;
        self
    }
}

/// Complete, const-friendly robot motion configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionConfig {
    pub tuning: DriveTuning,
    pub limits: MotionLimits,
    pub rates: RateFilters,
    pub settling: SettlingConfig,
    pub timing: MotionTiming,
    pub safety: ClearanceConfig,
    pub path: PathConfig,
    pub approach: ApproachConfig,
}

impl Default for MotionConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl MotionConfig {
    pub const DEFAULT: Self = Self {
        tuning: DriveTuning::STANDARD,
        limits: MotionLimits::DEFAULT,
        rates: RateFilters::DEFAULT,
        settling: SettlingConfig::DEFAULT,
        timing: MotionTiming::DEFAULT,
        safety: ClearanceConfig::DEFAULT,
        path: PathConfig::DEFAULT,
        approach: ApproachConfig::DEFAULT,
    };

    pub const fn new() -> Self {
        Self::DEFAULT
    }

    #[must_use]
    pub const fn with_limits(mut self, value: MotionLimits) -> Self {
        self.limits = value;
        self
    }

    #[must_use]
    pub const fn with_rates(mut self, value: RateFilters) -> Self {
        self.rates = value;
        self
    }

    #[must_use]
    pub const fn with_settling(mut self, value: SettlingConfig) -> Self {
        self.settling = value;
        self
    }

    #[must_use]
    pub const fn with_timing(mut self, value: MotionTiming) -> Self {
        self.timing = value;
        self
    }

    #[must_use]
    pub const fn with_safety(mut self, value: ClearanceConfig) -> Self {
        self.safety = value;
        self
    }

    #[must_use]
    pub const fn with_path(mut self, value: PathConfig) -> Self {
        self.path = value;
        self
    }

    #[must_use]
    pub const fn with_approach(mut self, value: ApproachConfig) -> Self {
        self.approach = value;
        self
    }

    #[must_use]
    pub const fn with_tuning(mut self, value: DriveTuning) -> Self {
        self.tuning = value;
        self
    }

    pub fn valid(self) -> bool {
        self.tuning.valid()
            && [
                self.limits.wheel_speed.value,
                self.limits.acceleration.value,
                self.limits.deceleration.value,
                self.limits.lateral_acceleration.value,
                self.limits.angular_speed.value,
                self.limits.angular_acceleration.value,
                self.limits.angular_deceleration.value,
                self.settling.position_tolerance.value,
                self.settling.heading_tolerance.value,
                self.settling.velocity_tolerance.value,
                self.settling.angular_velocity_tolerance.value,
                self.path.lookahead.value,
                self.path.maximum_cross_track_error.value,
                self.approach.reversal_heading.value,
                self.approach.reversal_distance_factor,
                self.approach.reversal_yaw_rate.value,
                self.approach.arc_correction_limit.value,
                self.approach.terminal_arc_correction_limit.value,
            ]
            .into_iter()
            .all(|v| v.is_finite() && v > 0.0)
            && self
                .limits
                .wheel_acceleration
                .is_none_or(|a| a.value.is_finite() && a.value > 0.0)
            && self.limits.minimum_linear_speed.value.is_finite()
            && self.limits.minimum_linear_speed.value >= 0.0
            && self.limits.minimum_angular_speed.value.is_finite()
            && self.limits.minimum_angular_speed.value >= 0.0
            && self.safety.clearance.value.is_finite()
            && self.safety.clearance.value >= 0.0
            && self
                .limits
                .forward_braking
                .is_none_or(|v| v.value.is_finite() && v.value > 0.0)
            && self
                .limits
                .yaw_braking
                .is_none_or(|v| v.value.is_finite() && v.value > 0.0)
            && (0.0..1.0).contains(&self.approach.boomerang_lead)
            && self.approach.reversal_heading.value > core::f64::consts::FRAC_PI_2
            && self.approach.reversal_heading.value <= core::f64::consts::PI
            && !self.timing.period.is_zero()
            && self.timing.maximum_gap >= self.timing.period
            && self.timing.progress_timeout >= self.timing.period
            && self.timing.stage_timeout >= self.timing.period
    }

    pub(crate) const fn options(self, track_width: Length) -> DriveOptions {
        DriveOptions::differential(track_width, self.limits.wheel_speed).with_settling(
            self.settling.position_tolerance,
            self.settling.heading_tolerance,
            self.settling.velocity_tolerance,
            self.settling.angular_velocity_tolerance,
            self.settling.settle,
        )
    }

    /// Apply the same prepared-trajectory constraints in firmware and simulation.
    pub(crate) fn constrained_for_path(
        mut self,
        mut options: DriveOptions,
        limits: ozton_math::path::Limits,
    ) -> (Self, DriveOptions) {
        self.limits.lateral_acceleration = self
            .limits
            .lateral_acceleration
            .min(limits.lateral_acceleration);
        self.limits.acceleration = self.limits.acceleration.min(limits.acceleration);
        self.limits.deceleration = self.limits.deceleration.min(limits.deceleration);
        self.limits.wheel_acceleration = Some(
            self.limits
                .wheel_acceleration
                .unwrap_or(self.limits.acceleration)
                .min(limits.wheel_acceleration),
        );
        options.maximum_wheel_speed = options.maximum_wheel_speed.min(limits.wheel_speed);
        (self, options)
    }

    pub fn path_limits(self, track_width: Length) -> ozton_math::path::Limits {
        ozton_math::path::Limits::from_units(
            track_width,
            self.limits.wheel_speed,
            self.limits.wheel_speed,
            self.limits.acceleration,
            self.limits.deceleration,
            self.limits.lateral_acceleration,
        )
        .with_wheel_acceleration(
            self.limits
                .wheel_acceleration
                .unwrap_or(self.limits.acceleration),
        )
    }
}
