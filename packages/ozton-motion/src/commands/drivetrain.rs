//! Tracked-drive commands that borrow only the drivetrain.
use std::time::Duration;

use ozton_drivetrain::{
    Drive, Drivetrain, Footprint,
    assist::{AssistedDrivetrain, DriveAssist},
    model::{Differential, DriveEncoders, WheelSpeedError},
};
use ozton_localization::{
    Fault,
    differential::{DifferentialLocalization, update_localization},
};
use ozton_math::{
    geometry::{DVec2, Pose},
    uom::si::f64::Velocity,
};
use ozton_tracking::sampling::{LocalizationLifecycle, SensorTracking};
use vexide::{competition::CompetitionMode, smart::PortError};

use super::{Feedback, MotionPlatform};

type TrackedDrive = Drivetrain<Differential, SensorTracking, DifferentialLocalization>;
type AssistedTrackedDrive<'a, A> =
    AssistedDrivetrain<'a, Differential, SensorTracking, DifferentialLocalization, A>;
const MAX_CONTROL_GAP: Duration = Duration::from_millis(100);
mod point;
mod pose_seeking;
pub use pose_seeking::PoseSeeking;

#[derive(Clone, Copy, Debug)]
pub enum DriveMotionFault {
    Tracking(Fault),
    Output(PortError),
    Brake(PortError),
    InvalidCommand,
}

impl std::fmt::Display for DriveMotionFault {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Tracking(fault) => write!(f, "Tracking feedback unavailable ({fault:?})"),
            Self::Output(error) => write!(f, "Could not command drive motors ({error})"),
            Self::Brake(error) => write!(f, "Could not stop drive motors ({error})"),
            Self::InvalidCommand => f.write_str("Invalid drivetrain command or wheel geometry"),
        }
    }
}

impl std::error::Error for DriveMotionFault {}

impl From<WheelSpeedError> for DriveMotionFault {
    fn from(error: WheelSpeedError) -> Self {
        match error {
            WheelSpeedError::Port(port) => Self::Output(port),
            _ => Self::InvalidCommand,
        }
    }
}

impl MotionPlatform for TrackedDrive {
    type Error = DriveMotionFault;

    fn sample(&mut self) -> Result<Option<Feedback>, Self::Error> {
        self.localization
            .set_competition_mode(CompetitionMode::Autonomous);
        let positions =
            if LocalizationLifecycle::uses_drive_encoders(&self.localization, &self.tracking) {
                self.tracking_positions()
            } else {
                [None; 2]
            };
        let width = self.track_width();
        update_localization(&mut self.localization, &mut self.tracking, positions, width)
            .map_err(DriveMotionFault::Tracking)
    }

    fn update_subsystems(&mut self, _: Duration) -> Result<(), Self::Error> {
        Ok(())
    }

    fn drive(&mut self, speeds: [Velocity; 2], _: Duration) -> Result<(), Self::Error> {
        self.drive_wheel_speeds(speeds).map_err(Into::into)
    }

    fn stop_drive(&mut self) -> Result<(), Self::Error> {
        self.stop_now().map_err(DriveMotionFault::Brake)
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.cancel_distance_reset();
        self.stop_drive()
    }

    fn set_pose(&mut self, pose: Pose) -> Result<(), Self::Error> {
        if self.localization.set_pose(pose) {
            Ok(())
        } else {
            Err(DriveMotionFault::InvalidCommand)
        }
    }

    fn pose_trusted(&self) -> bool {
        self.localization.pose_trusted()
    }

    #[cfg(feature = "logging")]
    fn log_step(
        &mut self,
        elapsed: Duration,
        dt: Duration,
        _: ozton_math::uom::si::f64::Length,
        _: [Velocity; 2],
    ) {
        use ozton_runtime::diagnostics::{SD_SAMPLE_INTERVAL, report_motion_drive};

        let period = SD_SAMPLE_INTERVAL.as_millis();
        if elapsed.as_millis() / period != elapsed.saturating_sub(dt).as_millis() / period {
            report_motion_drive(self);
        }
    }

    fn request_distance_reset(&mut self) {
        self.localization.request_distance_reset();
    }

    fn cancel_distance_reset(&mut self) {
        self.localization.cancel_distance_reset();
    }
}

/// Feedback and field geometry used by clearance-aware pose seeking.
pub trait PoseDrive: MotionPlatform<Error = DriveMotionFault> + DriveEncoders {
    fn pose(&self) -> Pose;
    fn footprint(&self) -> Option<Footprint>;
    fn rectangular_half_extents(&self) -> Option<DVec2>;
}

impl PoseDrive for TrackedDrive {
    fn pose(&self) -> Pose {
        self.localization.pose()
    }

    fn footprint(&self) -> Option<Footprint> {
        self.footprint()
    }

    fn rectangular_half_extents(&self) -> Option<DVec2> {
        self.localization.rectangular_half_extents()
    }
}

impl<A: DriveAssist<Differential, SensorTracking, DifferentialLocalization>> PoseDrive
    for AssistedTrackedDrive<'_, A>
{
    fn pose(&self) -> Pose {
        self.drivetrain().pose()
    }

    fn footprint(&self) -> Option<Footprint> {
        self.drivetrain().footprint()
    }

    fn rectangular_half_extents(&self) -> Option<DVec2> {
        self.drivetrain().rectangular_half_extents()
    }
}

impl<A: DriveAssist<Differential, SensorTracking, DifferentialLocalization>> MotionPlatform
    for AssistedTrackedDrive<'_, A>
{
    type Error = DriveMotionFault;

    fn sample(&mut self) -> Result<Option<Feedback>, Self::Error> {
        self.drivetrain_mut().sample()
    }

    fn update_subsystems(&mut self, dt: Duration) -> Result<(), Self::Error> {
        self.drivetrain_mut().update_subsystems(dt)
    }

    fn drive(&mut self, speeds: [Velocity; 2], _: Duration) -> Result<(), Self::Error> {
        self.drive_wheel_speeds(speeds).map_err(Into::into)
    }

    fn stop_drive(&mut self) -> Result<(), Self::Error> {
        self.stop_now().map_err(DriveMotionFault::Brake)
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.cancel_distance_reset();
        self.stop_drive()
    }

    fn set_pose(&mut self, pose: Pose) -> Result<(), Self::Error> {
        self.drivetrain_mut().set_pose(pose)
    }

    fn pose_trusted(&self) -> bool {
        self.drivetrain().pose_trusted()
    }

    #[cfg(feature = "logging")]
    fn log_step(
        &mut self,
        elapsed: Duration,
        dt: Duration,
        progress: ozton_math::uom::si::f64::Length,
        speeds: [Velocity; 2],
    ) {
        self.drivetrain_mut()
            .log_step(elapsed, dt, progress, speeds);
    }

    fn request_distance_reset(&mut self) {
        self.drivetrain_mut().request_distance_reset();
    }

    fn cancel_distance_reset(&mut self) {
        self.drivetrain_mut().cancel_distance_reset();
    }
}

/// Register a surveyed starting pose and align the IMU with the field.
pub trait ResetStartPose {
    fn reset(&mut self, starting_pose: Pose) -> Result<(), DriveMotionFault>;
}

impl ResetStartPose for TrackedDrive {
    fn reset(&mut self, starting_pose: Pose) -> Result<(), DriveMotionFault> {
        let heading = self
            .tracking
            .refresh_heading()
            .and_then(|sample| sample.heading);
        self.localization
            .register_start(starting_pose, heading)
            .then_some(())
            .ok_or(DriveMotionFault::InvalidCommand)
    }
}

impl<A> ResetStartPose for AssistedTrackedDrive<'_, A>
where
    A: DriveAssist<Differential, SensorTracking, DifferentialLocalization>,
{
    fn reset(&mut self, starting_pose: Pose) -> Result<(), DriveMotionFault> {
        self.stop_now().map_err(DriveMotionFault::Brake)?;
        self.drivetrain_mut().reset(starting_pose)
    }
}
