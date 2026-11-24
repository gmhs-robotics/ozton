//! Clearance-aware pose seeking for a calibrated differential drivetrain.

use std::time::Duration;

use ozton_math::{
    geometry::{DVec2, Pose},
    uom::si::f64::{Acceleration, Angle, AngularVelocity, Length, Velocity},
};

use crate::{
    basic::differential::{DriveOptions, DriveTuning},
    defaults::AUTON_STAGE_TIMEOUT,
};

mod approach;
mod drive;
mod error;
mod session;
pub use drive::{DriveMotionFault, PoseDrive};
pub use error::PoseMotionError;
use session::PoseSession;

pub const DEFAULT_STAGE_TIMEOUT: Duration = AUTON_STAGE_TIMEOUT;

#[derive(Clone, Copy, Debug)]
pub enum TravelDirection {
    Forward,
    Backward,
}

/// Reusable route settings for motion with live differential localization.
///
/// If the drivetrain has a footprint, a rectangular field is required for
/// planning a clear departure from a wall and checking in-place turns.
#[derive(Clone, Copy, Debug)]
pub struct PoseSeeking {
    pub(super) tuning: DriveTuning,
    pub(super) wheel_speed: Velocity,
    pub(super) position_tolerance: Option<Length>,
    pub(super) heading_tolerance: Option<Angle>,
    pub(super) travel: Option<TravelDirection>,
    pub(super) stage_timeout: Duration,
    pub(super) config: crate::MotionConfig,
}

impl Default for PoseSeeking {
    fn default() -> Self {
        Self::new()
    }
}

impl PoseSeeking {
    pub const fn new() -> Self {
        Self::from_config(crate::MotionConfig::DEFAULT)
    }

    /// Declare a reusable motion primitive directly from physical motion settings.
    pub const fn from_config(config: crate::MotionConfig) -> Self {
        Self {
            tuning: config.tuning,
            wheel_speed: config.wheel_speed,
            position_tolerance: Some(config.position_tolerance),
            heading_tolerance: Some(config.heading_tolerance),
            travel: None,
            stage_timeout: config.stage_timeout,
            config,
        }
    }

    /// Effective settings, including changes made through individual modifiers.
    pub const fn config(self) -> crate::MotionConfig {
        self.effective_config()
    }

    /// Apply a robot's physical constraints, PID gains and settling configuration.
    pub const fn with_config(mut self, config: crate::MotionConfig) -> Self {
        self.tuning = config.tuning;
        self.wheel_speed = config.wheel_speed;
        self.position_tolerance = Some(config.position_tolerance);
        self.heading_tolerance = Some(config.heading_tolerance);
        self.stage_timeout = config.stage_timeout;
        self.config = config;
        self
    }

    pub const fn with_linear_velocity_tolerance(mut self, tolerance: Velocity) -> Self {
        self.config.velocity_tolerance = tolerance;
        self
    }

    pub const fn with_angular_velocity_tolerance(
        mut self,
        tolerance: ozton_math::uom::si::f64::AngularVelocity,
    ) -> Self {
        self.config.angular_velocity_tolerance = tolerance;
        self
    }

    pub const fn with_tolerance_duration(mut self, duration: Duration) -> Self {
        self.config.settle = duration;
        self
    }

    pub const fn with_clearance(mut self, clearance: Length) -> Self {
        self.config.clearance = clearance;
        self
    }

    pub const fn with_departure_speed(mut self, speed: Velocity) -> Self {
        self.config.departure_speed = speed;
        self
    }

    pub(super) const fn effective_config(self) -> crate::MotionConfig {
        let mut config = self.config;
        config.tuning = self.tuning;
        config.wheel_speed = self.wheel_speed;
        config.position_tolerance = match self.position_tolerance { Some(value) => value, None => config.position_tolerance };
        config.heading_tolerance = match self.heading_tolerance { Some(value) => value, None => config.heading_tolerance };
        config.stage_timeout = self.stage_timeout;
        config
    }

    #[must_use]
    pub const fn with_acceleration(mut self, value: Acceleration) -> Self {
        self.config.acceleration = value;
        self
    }

    #[must_use]
    pub const fn with_deceleration(mut self, value: Acceleration) -> Self {
        self.config.deceleration = value;
        self
    }

    #[must_use]
    pub const fn with_lateral_acceleration(mut self, value: Acceleration) -> Self {
        self.config.lateral_acceleration = value;
        self
    }

    #[must_use]
    pub const fn with_angular_speed(mut self, value: AngularVelocity) -> Self {
        self.config.angular_speed = value;
        self
    }

    #[must_use]
    pub const fn with_angular_acceleration(mut self, value: f64) -> Self {
        self.config.angular_acceleration = value;
        self
    }

    #[must_use]
    pub const fn with_angular_deceleration(mut self, value: f64) -> Self {
        self.config.angular_deceleration = value;
        self
    }

    #[must_use]
    pub const fn with_linear_rate_filter(mut self, value: Duration) -> Self {
        self.config.linear_rate_filter = value;
        self
    }

    #[must_use]
    pub const fn with_angular_rate_filter(mut self, value: Duration) -> Self {
        self.config.angular_rate_filter = value;
        self
    }

    #[must_use]
    pub const fn with_period(mut self, value: Duration) -> Self {
        self.config.period = value;
        self
    }

    #[must_use]
    pub const fn with_maximum_gap(mut self, value: Duration) -> Self {
        self.config.maximum_gap = value;
        self
    }

    #[must_use]
    pub const fn with_progress_timeout(mut self, value: Duration) -> Self {
        self.config.progress_timeout = value;
        self
    }

    #[must_use]
    pub const fn with_departure_reserve(mut self, value: Length) -> Self {
        self.config.departure_reserve = value;
        self
    }

    #[must_use]
    pub const fn with_lookahead(mut self, value: Length) -> Self {
        self.config.lookahead = value;
        self
    }

    #[must_use]
    pub const fn with_ramsete_b(mut self, value: f64) -> Self {
        self.config.ramsete_b = value;
        self
    }

    #[must_use]
    pub const fn with_ramsete_zeta(mut self, value: f64) -> Self {
        self.config.ramsete_zeta = value;
        self
    }

    #[must_use]
    pub const fn with_realign_enter(mut self, value: Angle) -> Self {
        self.config.realign_enter = value;
        self
    }

    #[must_use]
    pub const fn with_realign_exit(mut self, value: Angle) -> Self {
        self.config.realign_exit = value;
        self
    }

    #[must_use]
    pub const fn with_reversal_heading(mut self, value: Angle) -> Self {
        self.config.reversal_heading = value;
        self
    }

    #[must_use]
    pub const fn with_reversal_distance_factor(mut self, value: f64) -> Self {
        self.config.reversal_distance_factor = value;
        self
    }

    #[must_use]
    pub const fn with_reversal_yaw_rate(mut self, value: AngularVelocity) -> Self {
        self.config.reversal_yaw_rate = value;
        self
    }

    #[must_use]
    pub const fn with_arc_correction_limit(mut self, value: AngularVelocity) -> Self {
        self.config.arc_correction_limit = value;
        self
    }

    #[must_use]
    pub const fn with_terminal_arc_correction_limit(mut self, value: AngularVelocity) -> Self {
        self.config.terminal_arc_correction_limit = value;
        self
    }

    #[must_use]
    pub const fn with_linear_controller(mut self, controller: ozton_math::control::Pid) -> Self {
        self.tuning.linear = controller;
        self
    }

    #[must_use]
    pub const fn with_linear_gains(mut self, kp: f64, ki: f64, kd: f64) -> Self {
        self.tuning.linear.set_gains(kp, ki, kd);
        self
    }

    #[must_use]
    pub const fn with_linear_ki(mut self, ki: f64) -> Self {
        self.tuning.linear.set_ki(ki);
        self
    }

    #[must_use]
    pub const fn with_linear_kd(mut self, kd: f64) -> Self {
        self.tuning.linear.set_kd(kd);
        self
    }

    #[must_use]
    pub const fn with_linear_integration_range(mut self, range: Length) -> Self {
        self.tuning.linear.set_integration_range(Some(range.value));
        self
    }

    #[must_use]
    pub const fn with_angular_controller(mut self, controller: ozton_math::control::Pid) -> Self {
        self.tuning.angular = controller;
        self
    }

    #[must_use]
    pub const fn with_angular_gains(mut self, kp: f64, ki: f64, kd: f64) -> Self {
        self.tuning.angular.set_gains(kp, ki, kd);
        self
    }

    #[must_use]
    pub const fn with_angular_ki(mut self, ki: f64) -> Self {
        self.tuning.angular.set_ki(ki);
        self
    }

    #[must_use]
    pub const fn with_angular_kd(mut self, kd: f64) -> Self {
        self.tuning.angular.set_kd(kd);
        self
    }

    #[must_use]
    pub const fn with_angular_integration_range(mut self, range: Angle) -> Self {
        self.tuning.angular.set_integration_range(Some(range.value));
        self
    }

    #[must_use]
    pub const fn with_linear_output_limit(mut self, limit: f64) -> Self {
        self.tuning.linear.set_output_limit(Some(limit));
        self
    }

    #[must_use]
    pub const fn without_linear_output_limit(mut self) -> Self {
        self.tuning.linear.set_output_limit(None);
        self
    }

    #[must_use]
    pub const fn without_linear_integration_range(mut self) -> Self {
        self.tuning.linear.set_integration_range(None);
        self
    }

    #[must_use]
    pub const fn with_angular_output_limit(mut self, limit: f64) -> Self {
        self.tuning.angular.set_output_limit(Some(limit));
        self
    }

    #[must_use]
    pub const fn without_angular_output_limit(mut self) -> Self {
        self.tuning.angular.set_output_limit(None);
        self
    }

    #[must_use]
    pub const fn without_angular_integration_range(mut self) -> Self {
        self.tuning.angular.set_integration_range(None);
        self
    }

    pub const fn with_tuning(mut self, tuning: DriveTuning) -> Self {
        self.tuning = tuning;
        self
    }

    pub const fn with_linear_kp(mut self, kp: f64) -> Self {
        self.tuning.linear.set_kp(kp);
        self
    }

    pub const fn with_angular_kp(mut self, kp: f64) -> Self {
        self.tuning.angular.set_kp(kp);
        self
    }

    pub const fn with_wheel_speed(mut self, wheel_speed: Velocity) -> Self {
        self.wheel_speed = wheel_speed;
        self
    }

    pub const fn with_position_tolerance(mut self, tolerance: Length) -> Self {
        self.position_tolerance = Some(tolerance);
        self
    }

    pub const fn with_heading_tolerance(mut self, tolerance: Angle) -> Self {
        self.heading_tolerance = Some(tolerance);
        self
    }

    /// Choose the direction of the approach after the clearance maneuver.
    /// By default the motion chooses the direction that requires less turning.
    pub const fn with_travel(mut self, travel: TravelDirection) -> Self {
        self.travel = Some(travel);
        self
    }

    pub const fn reverse(self) -> Self {
        self.with_travel(TravelDirection::Backward)
    }

    /// Set the timeout for each stage of the motion.
    pub const fn with_timeout(mut self, timeout: Duration) -> Self {
        self.stage_timeout = timeout;
        self
    }

    /// Run without a stage timeout while autonomous is active.
    pub const fn without_timeout(self) -> Self {
        self.with_timeout(Duration::MAX)
    }

    pub(super) fn options(
        self,
        drive: &impl ozton_drivetrain::model::DriveEncoders,
    ) -> Option<DriveOptions> {
        let config = self.effective_config();
        config.valid().then(|| ())?;
        Some(config.options(drive.track_width()?))
    }

    /// Move to a field point in metres without imposing a final heading.
    pub async fn move_to_point<D: PoseDrive>(
        &self,
        drive: &mut D,
        point: impl Into<DVec2>,
    ) -> Result<(), PoseMotionError<D::Error>> {
        approach::drive_to_target(drive, point.into(), None, *self).await
    }

    /// Move to a field pose and settle at its heading.
    pub async fn move_to_pose<D: PoseDrive>(
        &self,
        drive: &mut D,
        target: Pose,
    ) -> Result<(), PoseMotionError<D::Error>> {
        approach::drive_to_target(drive, target.position, Some(target.heading), *self).await
    }
    /// Drive a signed distance while preserving the starting heading.
    pub async fn drive_distance<D: PoseDrive>(
        &self,
        drive: &mut D,
        distance: Length,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let heading = drive.pose().heading;
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        if !distance.value.is_finite() {
            return Err(PoseMotionError::Configuration);
        }
        let field = approach::clearance(drive, self.config.clearance)?;
        let start = drive.pose();
        let end = Pose::new(
            start.position + ozton_math::geometry::direction(heading) * distance.value,
            heading,
        );
        if field.is_some_and(|f| !f.contains_pose(start) || !f.contains_pose(end)) {
            return Err(PoseMotionError::InsufficientClearance);
        }
        let mut session = PoseSession::new(drive, options, self.effective_config())?;
        let result = session
            .run(
                crate::basic::differential::DifferentialTarget::DriveDistance {
                    meters: distance,
                    heading: Some(heading),
                    maximum_speed: options.maximum_wheel_speed,
                },
                self.stage_timeout,
            )
            .await;
        session.finish(result)
    }

    /// Execute a constant-radius forward arc with a checked swept footprint.
    pub async fn drive_arc<D: PoseDrive>(
        &self,
        drive: &mut D,
        radius: Length,
        sweep: Angle,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        if !radius.value.is_finite()
            || radius.value <= 0.0
            || !sweep.value.is_finite()
            || sweep.value.abs() > core::f64::consts::TAU
        {
            return Err(PoseMotionError::Configuration);
        }
        if let Some(field) = approach::clearance(drive, self.config.clearance)? {
            let footprint = drive.footprint().ok_or(PoseMotionError::Configuration)?;
            let (min, max) = footprint
                .forward_arc_bounds(drive.pose(), radius, sweep)
                .ok_or(PoseMotionError::Configuration)?;
            if !field.contains_aabb(min, max) {
                return Err(PoseMotionError::InsufficientClearance);
            }
        }
        let mut session = PoseSession::new(drive, options, self.effective_config())?;
        let result = session
            .run(
                crate::basic::differential::DifferentialTarget::DriveArc {
                    radius,
                    sweep,
                    maximum_speed: options.maximum_wheel_speed,
                },
                self.stage_timeout,
            )
            .await;
        session.finish(result)
    }

    pub async fn turn_to_heading<D: PoseDrive>(
        &self,
        drive: &mut D,
        heading: Angle,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        if !heading.value.is_finite() {
            return Err(PoseMotionError::Configuration);
        }
        if approach::clearance(drive, self.config.clearance)?
            .is_some_and(|f| !f.can_turn(drive.pose()))
        {
            return Err(PoseMotionError::InsufficientClearance);
        }
        let mut session = PoseSession::new(drive, options, self.effective_config())?;
        let result = session
            .run(
                crate::basic::differential::DifferentialTarget::Turn { heading },
                self.stage_timeout,
            )
            .await;
        session.finish(result)
    }

    pub async fn follow_path<D: PoseDrive>(
        &self,
        drive: &mut D,
        path: &ozton_math::path::Path,
        follower: crate::pursuit::profiled::Follower,
        reverse: bool,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        let field = approach::clearance(drive, self.config.clearance)?;
        let facing = ozton_math::angle!(if reverse { 180.0 } else { 0.0 }, deg);
        let start = path.start().pose;
        let actual = drive.pose();
        if !follower.is_valid()
            || actual.position.distance(start.position) > 0.08
            || ozton_math::geometry::wrap_heading(start.heading + facing - actual.heading)
                .value
                .abs()
                > 0.35
        {
            return Err(PoseMotionError::Configuration);
        }
        if field.is_some_and(|f| !ozton_math::path::planner::clear(path, reverse, f)) {
            return Err(PoseMotionError::InsufficientClearance);
        }
        let end = path.end().pose;
        let target = Pose::new(end.position, end.heading + facing);
        let mut session = PoseSession::new(drive, options, self.effective_config())?;
        let result = session
            .follow(path, follower, reverse, target, field, self.stage_timeout)
            .await;
        session.finish(result)
    }

    /// Hold the chassis still and try to correct the pose from fresh wall ranges.
    pub async fn try_distance_reset<D: PoseDrive>(
        &self,
        drive: &mut D,
        timeout: Duration,
    ) -> Result<bool, PoseMotionError<D::Error>> {
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        let mut session = PoseSession::new(drive, options, self.effective_config())?;
        let result = session.try_distance_reset(timeout).await;
        session.finish(result)
    }
}
