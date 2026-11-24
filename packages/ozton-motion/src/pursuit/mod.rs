//! Adaptive pure pursuit controller.

use std::time::Duration;

#[cfg(feature = "vex")]
use ozton_drivetrain::{Drive, HasLocalization};
use ozton_math::{control::Pid, uom::si::f64::{AngularVelocity, Length, Velocity}};
#[cfg(feature = "vex")]
use ozton_math::{
    localization::observation::{TracksHeading, TracksPosition},
    path::Waypoint,
};

/// Normalized-output endpoint controller for the Evian-compatible pursuit API.
/// Gains output motor fractions, not physical wheel velocities.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PursuitTerminalConfig {
    pub linear: Pid,
    pub angular: Pid,
    pub position_tolerance: Length,
    pub velocity_tolerance: Velocity,
    pub angular_velocity_tolerance: AngularVelocity,
    pub settle: Duration,
    pub rate_filter: Duration,
    pub period: Duration,
}

impl Default for PursuitTerminalConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl PursuitTerminalConfig {
    pub const fn new() -> Self {
        let mut linear = Pid::new(2.0, 0.0, 0.10, None);
        let mut angular = Pid::new(1.5, 0.0, 0.06, None);
        linear.set_output_limit(Some(0.35));
        angular.set_output_limit(Some(0.40));
        Self {
            linear,
            angular,
            position_tolerance: crate::defaults::AUTON_POSITION_TOLERANCE,
            velocity_tolerance: ozton_math::qty!(0.04 m/s),
            angular_velocity_tolerance: ozton_math::qty!(0.12 rad/s),
            settle: Duration::from_millis(120),
            rate_filter: Duration::from_millis(40),
            period: Duration::from_millis(10),
        }
    }

    #[must_use]
    pub const fn with_linear_controller(mut self, value: Pid) -> Self {
        self.linear = value;
        self
    }

    #[must_use]
    pub const fn with_angular_controller(mut self, value: Pid) -> Self {
        self.angular = value;
        self
    }

    #[must_use]
    pub const fn with_position_tolerance(mut self, value: Length) -> Self {
        self.position_tolerance = value;
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
    pub const fn with_rate_filter(mut self, value: Duration) -> Self {
        self.rate_filter = value;
        self
    }

    #[must_use]
    pub const fn with_period(mut self, value: Duration) -> Self {
        self.period = value;
        self
    }

    pub fn valid(self) -> bool {
        crate::DriveTuning {
            linear: self.linear,
            angular: self.angular,
        }
        .valid()
            && [
                self.position_tolerance.value,
                self.velocity_tolerance.value,
                self.angular_velocity_tolerance.value,
            ]
            .into_iter()
            .all(|value| value.is_finite() && value > 0.0)
            && !self.period.is_zero()
    }
}

#[cfg(feature = "vex")]
mod follow;
#[cfg(feature = "vex")]
pub use follow::PurePursuitFuture;

/// Adaptive pure pursuit controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PurePursuit {
    /// Radius of the robot's lookahead circle.
    pub lookahead_distance: Length,

    /// Distance between the left and right wheels on the robot.
    pub track_width: Length,

    /// Maximum duration the motion can take before being cancelled.
    pub timeout: Option<Duration>,
    /// Feedback used when settling at the final waypoint.
    pub terminal_config: PursuitTerminalConfig,
}

impl Default for PurePursuit {
    fn default() -> Self {
        Self::new()
    }
}

impl PurePursuit {
    /// Set the actual drivetrain track width with `with_track_width` before following.
    pub const fn new() -> Self {
        Self {
            lookahead_distance: ozton_math::length!(0.16 m),
            track_width: ozton_math::length!(0 m),
            timeout: Some(crate::defaults::AUTON_STAGE_TIMEOUT),
            terminal_config: PursuitTerminalConfig::new(),
        }
    }

    #[must_use]
    pub const fn with_lookahead_distance(mut self, value: Length) -> Self {
        self.lookahead_distance = value;
        self
    }

    #[must_use]
    pub const fn with_track_width(mut self, value: Length) -> Self {
        self.track_width = value;
        self
    }

    #[must_use]
    pub const fn with_timeout(mut self, value: Duration) -> Self {
        self.timeout = Some(value);
        self
    }

    pub const fn without_timeout(mut self) -> Self {
        self.timeout = None;
        self
    }

    #[must_use]
    pub const fn with_terminal_config(mut self, config: PursuitTerminalConfig) -> Self {
        self.terminal_config = config;
        self
    }

    #[must_use]
    pub const fn with_linear_controller(mut self, controller: Pid) -> Self {
        self.terminal_config.linear = controller;
        self
    }

    #[must_use]
    pub const fn with_angular_controller(mut self, controller: Pid) -> Self {
        self.terminal_config.angular = controller;
        self
    }

    #[must_use]
    pub const fn with_linear_gains(mut self, kp: f64, ki: f64, kd: f64) -> Self {
        self.terminal_config.linear.set_gains(kp, ki, kd);
        self
    }

    #[must_use]
    pub const fn with_angular_gains(mut self, kp: f64, ki: f64, kd: f64) -> Self {
        self.terminal_config.angular.set_gains(kp, ki, kd);
        self
    }

    #[must_use]
    pub const fn with_position_tolerance(mut self, value: Length) -> Self {
        self.terminal_config.position_tolerance = value;
        self
    }

    #[must_use]
    pub const fn with_linear_velocity_tolerance(mut self, value: Velocity) -> Self {
        self.terminal_config.velocity_tolerance = value;
        self
    }

    #[must_use]
    pub const fn with_angular_velocity_tolerance(mut self, value: AngularVelocity) -> Self {
        self.terminal_config.angular_velocity_tolerance = value;
        self
    }

    #[must_use]
    pub const fn with_tolerance_duration(mut self, value: Duration) -> Self {
        self.terminal_config.settle = value;
        self
    }

    #[must_use]
    pub const fn with_rate_filter(mut self, value: Duration) -> Self {
        self.terminal_config.rate_filter = value;
        self
    }

    #[must_use]
    pub const fn with_period(mut self, value: Duration) -> Self {
        self.terminal_config.period = value;
        self
    }
}

#[cfg(feature = "vex")]
impl PurePursuit {
    /// Moves a drivetrain along a set of discrete waypoints using pure pursuit.
    pub fn follow<
        'a,
        I: Iterator<Item = Waypoint> + Unpin,
        M: Drive + HasLocalization<Localization: TracksPosition + TracksHeading>,
    >(
        &self,
        drivetrain: &'a mut M,
        waypoints: impl IntoIterator<Item = Waypoint, IntoIter = I>,
    ) -> PurePursuitFuture<'a, M, I> {
        PurePursuitFuture {
            drivetrain,
            state: None,
            waypoints: waypoints.into_iter(),
            lookahead_distance: self.lookahead_distance.value,
            track_width: self.track_width.value,
            timeout: self.timeout,
            terminal_config: self.terminal_config,
        }
    }
}

pub mod profiled;
