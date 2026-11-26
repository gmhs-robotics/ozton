//! Pose feedback for a calibrated differential drivetrain.

use std::time::{Duration, Instant};

use ozton_localization::LocalizationEstimate;
use ozton_math::{
    geometry::{DVec2, Pose},
    uom::si::f64::{Angle, Length},
};

use crate::basic::differential::DriveOptions;

mod actions;
mod approach;
mod chain;
pub(super) use chain::Command;
pub use chain::{MotionChain, MotionSegment, RunMotion};
mod error;
mod session;
pub use actions::PathActions;
pub use error::PoseMotionError;
pub use ozton_drivetrain::{
    BrakeMode, MotionExecution, RangeResetDrive, WheelActuation, WheelCommand,
};
pub use ozton_math::path::clearance::WallContact;
use session::{PoseSession, Trajectory};

use super::Engine;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TravelDirection {
    Forward,
    Backward,
}

impl Engine {
    /// Hold a stopped drivetrain while localization settles. Warn once on low
    /// confidence; cancellation remains an error.
    pub async fn settle<D: MotionExecution>(
        &self,
        drive: &mut D,
        duration: Duration,
    ) -> Result<(), PoseMotionError<D::Error>> {
        self.options(drive).ok_or(PoseMotionError::Configuration)?;
        drive.brake().map_err(PoseMotionError::StopFailed)?;
        let started = Instant::now();
        let mut warned = false;
        while started.elapsed() < duration {
            if !drive.motion_active() {
                return Err(PoseMotionError::Cancelled);
            }
            let uncertain = !drive.localization().pose_trusted();
            if !warned && uncertain {
                warned = true;
                ozton_diagnostics::log_event!(
                    Warn,
                    "localization",
                    "Initial pose confidence is low"
                );
            }
            drive
                .wait_until(Instant::now() + self.config.timing.period)
                .await;
        }
        Ok(())
    }

    pub(super) fn options(
        self,
        drive: &impl ozton_drivetrain::model::DriveEncoders,
    ) -> Option<DriveOptions> {
        let config = self.config;
        config.valid().then(|| ())?;
        Some(config.options(drive.track_width()))
    }

    /// Direct distance/bearing PID seeking without geometric path search.
    pub async fn move_to_point<D: MotionExecution>(
        &self,
        drive: &mut D,
        point: impl Into<DVec2>,
    ) -> Result<(), PoseMotionError<D::Error>> {
        approach::drive_to_target(drive, point.into(), None, *self).await
    }

    /// Approach an absolute pose using continuous Boomerang feedback.
    pub async fn move_to_pose<D: MotionExecution>(
        &self,
        drive: &mut D,
        target: Pose,
    ) -> Result<(), PoseMotionError<D::Error>> {
        if !target.is_finite() {
            return Err(PoseMotionError::Configuration);
        }
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        let reverse = self.travel.map_or_else(
            || {
                ozton_math::geometry::wrap_heading(
                    ozton_math::geometry::bearing(
                        target.position - drive.localization().pose().position,
                    ) - drive.localization().pose().heading,
                )
                .value
                .abs()
                    > core::f64::consts::FRAC_PI_2
            },
            |direction| direction == TravelDirection::Backward,
        );
        let mut session = PoseSession::new(drive, options, self.config)?;
        let result = session
            .run(
                crate::basic::differential::DifferentialTarget::DriveToPose {
                    target,
                    reverse,
                    maximum_speed: options.maximum_wheel_speed,
                },
                self.config.timing.stage_timeout,
            )
            .await;
        session.finish(result)
    }

    /// Rotate, drive straight to a position, then settle at its absolute heading.
    pub async fn move_to_pose_straight<D: MotionExecution>(
        &self,
        drive: &mut D,
        target: Pose,
    ) -> Result<(), PoseMotionError<D::Error>> {
        approach::drive_to_target(drive, target.position, Some(target.heading), *self).await
    }

    /// Drive a signed distance while preserving the starting heading.
    pub async fn drive_distance<D: MotionExecution>(
        &self,
        drive: &mut D,
        distance: Length,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let heading = drive.localization().pose().heading;
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        if !distance.value.is_finite() {
            return Err(PoseMotionError::Configuration);
        }
        // The surveyed contact is a stationary localization prior, not a motion limit.
        drive.localization_mut().release_wall_contact();
        let mut session = PoseSession::new(drive, options, self.config)?;
        let result = session
            .run(
                crate::basic::differential::DifferentialTarget::DriveDistance {
                    meters: distance,
                    heading: Some(heading),
                    maximum_speed: options.maximum_wheel_speed,
                },
                self.config.timing.stage_timeout,
            )
            .await;
        session.finish(result)
    }

    /// Execute a constant-radius forward arc.
    pub async fn drive_arc<D: MotionExecution>(
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
        let mut session = PoseSession::new(drive, options, self.config)?;
        let result = session
            .run(
                crate::basic::differential::DifferentialTarget::DriveArc {
                    radius,
                    sweep,
                    maximum_speed: options.maximum_wheel_speed,
                },
                self.config.timing.stage_timeout,
            )
            .await;
        session.finish(result)
    }

    /// Turn to face a field point using the physical feedback controller.
    pub async fn turn_to_point<D: MotionExecution>(
        &self,
        drive: &mut D,
        target: impl Into<DVec2>,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let target = target.into();
        if !target.is_finite() {
            return Err(PoseMotionError::Configuration);
        }
        self.turn_target(
            drive,
            crate::basic::differential::DifferentialTarget::TurnToPoint { target },
        )
        .await
    }

    pub async fn turn_to_heading<D: MotionExecution>(
        &self,
        drive: &mut D,
        heading: Angle,
    ) -> Result<(), PoseMotionError<D::Error>> {
        if !heading.value.is_finite() {
            return Err(PoseMotionError::Configuration);
        }
        self.turn_target(
            drive,
            crate::basic::differential::DifferentialTarget::Turn { heading },
        )
        .await
    }

    /// Forward swing to an absolute heading with the inside wheel group held.
    /// Only drives with explicit mixed braking/velocity support can use it.
    pub async fn swing_to_heading<D: MotionExecution + WheelActuation>(
        &self,
        drive: &mut D,
        heading: Angle,
    ) -> Result<(), PoseMotionError<D::Error>>
    where
        D::Error: From<D::WheelError>,
    {
        if !heading.value.is_finite() {
            return Err(PoseMotionError::Configuration);
        }
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        let start = drive.localization().pose();
        let sweep = ozton_math::geometry::wrap_heading(heading - start.heading);
        let hold_left = sweep.value >= 0.0;
        let held = usize::from(!hold_left);
        let mut session = PoseSession::new(drive, options, self.config)?;
        let result = session
            .run_with_output(
                crate::basic::differential::DifferentialTarget::Swing { heading, hold_left },
                self.config.timing.stage_timeout,
                |drive, speeds| {
                    let mut commands = speeds.map(WheelCommand::Velocity);
                    commands[held] = WheelCommand::Brake(BrakeMode::Hold);
                    drive.drive_wheel_commands(commands).map_err(Into::into)
                },
            )
            .await;
        session.finish(result)
    }

    async fn turn_target<D: MotionExecution>(
        &self,
        drive: &mut D,
        target: crate::basic::differential::DifferentialTarget,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        let mut session = PoseSession::new(drive, options, self.config)?;
        let result = session.run(target, self.config.timing.stage_timeout).await;
        session.finish(result)
    }

    /// Load a host-generated path after checking that its limits match the robot.
    pub fn prepare_path<D: MotionExecution>(
        &self,
        drive: &D,
        samples: &'static [ozton_math::path::PathSample],
        generated_limits: [f64; 7],
    ) -> Result<ozton_math::path::Path, PoseMotionError<D::Error>> {
        let limits = self.config.path_limits(drive.track_width());
        if !limits.matches(generated_limits) {
            return Err(PoseMotionError::Configuration);
        }
        ozton_math::path::Path::from_precomputed(samples, limits)
            .map_err(|_| PoseMotionError::Configuration)
    }

    /// Execute a continuous trajectory with nonblocking mechanism actions.
    pub async fn follow_with_actions<D: MotionExecution>(
        &self,
        drive: &mut D,
        path: &ozton_math::path::Path,
        reverse: bool,
        actions: &mut PathActions<'_>,
    ) -> Result<(), PoseMotionError<D::Error>> {
        if !actions.valid(path) {
            return Err(PoseMotionError::Configuration);
        }
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        let facing = ozton_math::qty!((if reverse { 180.0 } else { 0.0 }) deg);
        let start = path.start().pose;
        let actual = drive.localization().pose();
        if !actual.is_finite() {
            return Err(PoseMotionError::Configuration);
        }
        if actual.position.distance(start.position) > 0.08
            || ozton_math::geometry::wrap_heading(start.heading + facing - actual.heading)
                .value
                .abs()
                > 0.35
        {
            ozton_diagnostics::log_event!(
                Warn,
                "motion",
                "Trajectory start offset exceeds tuning threshold; continuing feedback"
            );
        }
        let end = path.end().pose;
        let target = Pose::new(end.position, end.heading + facing);
        drive.localization_mut().release_wall_contact();
        let mut session = PoseSession::new(drive, options, self.config)?;
        let result = session
            .follow(
                Trajectory {
                    path,
                    reverse,
                    target,
                    timeout: self.config.timing.stage_timeout,
                },
                actions,
            )
            .await;
        session.finish(result)
    }

    /// Hold the chassis still and try to correct the pose from fresh wall ranges.
    pub async fn try_distance_reset<D: RangeResetDrive>(
        &self,
        drive: &mut D,
        timeout: Duration,
    ) -> Result<bool, PoseMotionError<D::Error>> {
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        let mut session = PoseSession::new(drive, options, self.config)?;
        let before = session.drive.localization().pose();
        let result = session.try_distance_reset(timeout).await;
        let after = session.drive.localization().pose();
        if let Ok(accepted) = &result {
            ozton_diagnostics::log_row!(
                Info,
                "DISTANCE_RESET_RESULT",
                "accepted={accepted},before_x={:.6},before_y={:.6},after_x={:.6},after_y={:.6},delta_m={:.6}",
                before.position.x,
                before.position.y,
                after.position.x,
                after.position.y,
                before.position.distance(after.position)
            );
        }
        session.finish(result)
    }
}
