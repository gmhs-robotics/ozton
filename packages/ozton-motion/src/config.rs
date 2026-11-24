//! Robot-specific physical motion configuration. All scalar dynamics use SI units.
use std::time::Duration;

use ozton_math::uom::si::f64::{Acceleration, Angle, AngularAcceleration, AngularVelocity, Length, Velocity};

use crate::basic::differential::{DriveOptions, DriveTuning};

/// Defaults are deliberately conservative; applications should supply measured values.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionConfig {
    pub tuning: DriveTuning,
    pub wheel_speed: Velocity,
    pub acceleration: Acceleration,
    pub deceleration: Acceleration,
    pub lateral_acceleration: Acceleration,
    pub angular_speed: AngularVelocity,
    /// rad/s²
    pub angular_acceleration: AngularAcceleration,
    /// rad/s²
    pub angular_deceleration: AngularAcceleration,
    pub linear_rate_filter: Duration,
    pub angular_rate_filter: Duration,
    pub position_tolerance: Length,
    pub heading_tolerance: Angle,
    pub velocity_tolerance: Velocity,
    pub angular_velocity_tolerance: AngularVelocity,
    pub settle: Duration,
    pub period: Duration,
    pub maximum_gap: Duration,
    pub progress_timeout: Duration,
    pub stage_timeout: Duration,
    pub clearance: Length,
    pub departure_reserve: Length,
    pub departure_speed: Velocity,
    pub lookahead: Length,
    pub ramsete_b: f64,
    pub ramsete_zeta: f64,
    pub realign_enter: Angle,
    pub realign_exit: Angle,
    pub reversal_heading: Angle,
    pub reversal_distance_factor: f64,
    pub reversal_yaw_rate: AngularVelocity,
    pub arc_correction_limit: AngularVelocity,
    pub terminal_arc_correction_limit: AngularVelocity,
}

impl Default for MotionConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl MotionConfig {
    pub const DEFAULT: Self = Self {
        tuning: DriveTuning::STANDARD,
        wheel_speed: ozton_math::units::qty!(0.50 m/s),
        acceleration: ozton_math::units::qty!(0.8 m/s^2),
        deceleration: ozton_math::units::qty!(2.0 m/s^2),
        lateral_acceleration: ozton_math::units::qty!(0.8 m/s^2),
        angular_speed: ozton_math::units::qty!(5.0 rad/s),
        angular_acceleration: ozton_math::qty!(8.0 rad/s^2),
        angular_deceleration: ozton_math::qty!(8.0 rad/s^2),
        linear_rate_filter: Duration::from_millis(55),
        angular_rate_filter: Duration::from_millis(55),
        position_tolerance: ozton_math::qty!(0.025 m),
        heading_tolerance: ozton_math::qty!(0.06 rad),
        velocity_tolerance: ozton_math::units::qty!(0.04 m/s),
        angular_velocity_tolerance: ozton_math::units::qty!(0.12 rad/s),
        settle: Duration::from_millis(120),
        period: Duration::from_millis(10),
        maximum_gap: Duration::from_millis(100),
        progress_timeout: Duration::from_millis(2500),
        stage_timeout: Duration::from_secs(8),
        clearance: ozton_math::qty!(0.5 in),
        departure_reserve: ozton_math::qty!(0.25 in),
        departure_speed: ozton_math::units::qty!(0.30 m/s),
        lookahead: ozton_math::qty!(0.16 m),
        ramsete_b: 2.0,
        ramsete_zeta: 0.7,
        realign_enter: ozton_math::qty!(35 deg),
        realign_exit: ozton_math::qty!(12 deg),
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
    pub const fn with_tuning(mut self, value: DriveTuning) -> Self {
        self.tuning = value;
        self
    }

    #[must_use]
    pub const fn with_wheel_speed(mut self, value: Velocity) -> Self {
        self.wheel_speed = value;
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
    pub const fn with_linear_velocity_tolerance(mut self, value: Velocity) -> Self {
        self.velocity_tolerance = value;
        self
    }

    #[must_use]
    pub const fn with_angular_velocity_tolerance(mut self, value: AngularVelocity) -> Self {
        self.angular_velocity_tolerance = value;
        self
    }

    #[must_use]
    pub const fn with_tolerance_duration(mut self, value: Duration) -> Self {
        self.settle = value;
        self
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
    pub const fn with_timeout(mut self, value: Duration) -> Self {
        self.stage_timeout = value;
        self
    }

    #[must_use]
    pub const fn with_clearance(mut self, value: Length) -> Self {
        self.clearance = value;
        self
    }

    #[must_use]
    pub const fn with_departure_reserve(mut self, value: Length) -> Self {
        self.departure_reserve = value;
        self
    }

    #[must_use]
    pub const fn with_departure_speed(mut self, value: Velocity) -> Self {
        self.departure_speed = value;
        self
    }

    #[must_use]
    pub const fn with_lookahead_distance(mut self, value: Length) -> Self {
        self.lookahead = value;
        self
    }

    #[must_use]
    pub const fn with_ramsete_b(mut self, value: f64) -> Self {
        self.ramsete_b = value;
        self
    }

    #[must_use]
    pub const fn with_ramsete_zeta(mut self, value: f64) -> Self {
        self.ramsete_zeta = value;
        self
    }

    #[must_use]
    pub const fn with_realign_enter(mut self, value: Angle) -> Self {
        self.realign_enter = value;
        self
    }

    #[must_use]
    pub const fn with_realign_exit(mut self, value: Angle) -> Self {
        self.realign_exit = value;
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

    pub fn valid(self) -> bool {
        self.tuning.valid()
            && [
                self.wheel_speed.value,
                self.acceleration.value,
                self.deceleration.value,
                self.lateral_acceleration.value,
                self.angular_speed.value,
                self.angular_acceleration.value,
                self.angular_deceleration.value,
                self.position_tolerance.value,
                self.heading_tolerance.value,
                self.velocity_tolerance.value,
                self.angular_velocity_tolerance.value,
                self.departure_speed.value,
                self.lookahead.value,
                self.ramsete_b,
                self.ramsete_zeta,
                self.realign_enter.value,
                self.realign_exit.value,
                self.reversal_heading.value,
                self.reversal_distance_factor,
                self.reversal_yaw_rate.value,
                self.arc_correction_limit.value,
                self.terminal_arc_correction_limit.value,
            ]
            .into_iter()
            .all(|v| v.is_finite() && v > 0.0)
            && [self.clearance.value, self.departure_reserve.value]
                .into_iter()
                .all(|v| v.is_finite() && v >= 0.0)
            && self.ramsete_zeta <= 1.0
            && self.realign_exit < self.realign_enter
            && self.realign_enter.value < core::f64::consts::FRAC_PI_2
            && self.reversal_heading.value > core::f64::consts::FRAC_PI_2
            && self.reversal_heading.value <= core::f64::consts::PI
            && !self.period.is_zero()
            && self.maximum_gap >= self.period
            && self.progress_timeout >= self.period
            && self.stage_timeout >= self.period
    }

    pub const fn options(self, track_width: Length) -> DriveOptions {
        DriveOptions::differential(track_width, self.wheel_speed).with_settling(
            self.position_tolerance,
            self.heading_tolerance,
            self.velocity_tolerance,
            self.angular_velocity_tolerance,
            self.settle,
        )
    }

    pub fn path_limits(self, track_width: Length) -> ozton_math::path::Limits {
        ozton_math::path::Limits::from_units(
            track_width,
            self.wheel_speed,
            self.wheel_speed,
            self.acceleration,
            self.deceleration,
            self.lateral_acceleration,
        )
    }

    pub const fn follower(self) -> crate::pursuit::profiled::Follower {
        crate::pursuit::profiled::Follower::Ramsete {
            b: self.ramsete_b,
            zeta: self.ramsete_zeta,
        }
    }
}
