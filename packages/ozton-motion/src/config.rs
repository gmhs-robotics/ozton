//! Robot-specific physical motion configuration. All scalar dynamics use SI units.
use std::time::Duration;

use ozton_math::uom::si::f64::{Acceleration, Angle, AngularVelocity, Length, Velocity};

use crate::basic::differential::{DriveOptions, DriveTuning};

/// Defaults are deliberately conservative; applications should supply measured values.
#[derive(Debug, Clone, Copy)]
pub struct MotionConfig {
    pub tuning: DriveTuning,
    pub wheel_speed: Velocity,
    pub acceleration: Acceleration,
    pub deceleration: Acceleration,
    pub lateral_acceleration: Acceleration,
    pub angular_speed: AngularVelocity,
    /// rad/s²
    pub angular_acceleration: f64,
    /// rad/s²
    pub angular_deceleration: f64,
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
        Self {
            tuning: DriveTuning::STANDARD,
            wheel_speed: ozton_math::units::velocity!(0.50, meter_per_second),
            acceleration: Acceleration::new::<
                ozton_math::uom::si::acceleration::meter_per_second_squared,
            >(0.8),
            deceleration: Acceleration::new::<
                ozton_math::uom::si::acceleration::meter_per_second_squared,
            >(2.0),
            lateral_acceleration: Acceleration::new::<
                ozton_math::uom::si::acceleration::meter_per_second_squared,
            >(0.8),
            angular_speed: ozton_math::units::angular_velocity!(5.0, radian_per_second),
            angular_acceleration: 8.0,
            angular_deceleration: 8.0,
            linear_rate_filter: Duration::from_millis(55),
            angular_rate_filter: Duration::from_millis(55),
            position_tolerance: ozton_math::length!(0.025 m),
            heading_tolerance: ozton_math::angle!(0.06 rad),
            velocity_tolerance: ozton_math::units::velocity!(0.04, meter_per_second),
            angular_velocity_tolerance: ozton_math::units::angular_velocity!(
                0.12,
                radian_per_second
            ),
            settle: Duration::from_millis(120),
            period: Duration::from_millis(10),
            maximum_gap: Duration::from_millis(100),
            progress_timeout: Duration::from_millis(2500),
            stage_timeout: Duration::from_secs(8),
            clearance: ozton_math::length!(0.5 in),
            departure_reserve: ozton_math::length!(0.25 in),
            departure_speed: ozton_math::units::velocity!(0.30, meter_per_second),
            lookahead: ozton_math::length!(0.16 m),
            ramsete_b: 2.0,
            ramsete_zeta: 0.7,
            realign_enter: ozton_math::angle!(35 deg),
            realign_exit: ozton_math::angle!(12 deg),
            reversal_heading: ozton_math::angle!(125 deg),
            reversal_distance_factor: 4.0,
            reversal_yaw_rate: ozton_math::units::angular_velocity!(0.5, radian_per_second),
            arc_correction_limit: ozton_math::units::angular_velocity!(0.35, radian_per_second),
            terminal_arc_correction_limit: ozton_math::units::angular_velocity!(
                0.5,
                radian_per_second
            ),
        }
    }
}

impl MotionConfig {
    pub fn valid(self) -> bool {
        self.tuning.valid()
            && [
                self.wheel_speed.value,
                self.acceleration.value,
                self.deceleration.value,
                self.lateral_acceleration.value,
                self.angular_speed.value,
                self.angular_acceleration,
                self.angular_deceleration,
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

    pub fn options(self, track_width: Length) -> DriveOptions {
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

    pub fn follower(self) -> crate::pursuit::profiled::Follower {
        crate::pursuit::profiled::Follower::Ramsete {
            b: self.ramsete_b,
            zeta: self.ramsete_zeta,
        }
    }
}
