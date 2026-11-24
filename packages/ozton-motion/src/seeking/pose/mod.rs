//! Clearance-aware pose seeking for a calibrated differential drivetrain.

use std::time::Duration;

use ozton_math::{
    geometry::{DVec2, Pose},
    uom::si::{
        f64::{Angle, Length, Velocity},
        velocity::meter_per_second,
    },
};

use crate::{
    basic::differential::{DriveOptions, DriveTuning},
    defaults::{AUTON_SPEED, AUTON_STAGE_TIMEOUT},
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
}

impl Default for PoseSeeking {
    fn default() -> Self {
        Self {
            tuning: DriveTuning::STANDARD,
            wheel_speed: Velocity {
                value: 0.50,
                ..AUTON_SPEED
            },
            position_tolerance: None,
            heading_tolerance: None,
            travel: None,
            stage_timeout: AUTON_STAGE_TIMEOUT,
        }
    }
}

impl PoseSeeking {
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
        let mut options = DriveOptions::for_differential(drive, self.wheel_speed)?;
        options.position_tolerance = self
            .position_tolerance
            .unwrap_or(options.position_tolerance);
        options.heading_tolerance = self.heading_tolerance.unwrap_or(options.heading_tolerance);
        Some(options)
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
    /// Hold the chassis still and try to correct the pose from fresh wall ranges.
    pub async fn try_distance_reset<D: PoseDrive>(
        &self,
        drive: &mut D,
        timeout: Duration,
    ) -> Result<bool, PoseMotionError<D::Error>> {
        let options = DriveOptions::for_differential(drive, Velocity::new::<meter_per_second>(1.0))
            .ok_or(PoseMotionError::Configuration)?;
        let mut session = PoseSession::new(drive, options, self.tuning)?;
        let result = session.try_distance_reset(timeout).await;
        session.finish(result)
    }
}
