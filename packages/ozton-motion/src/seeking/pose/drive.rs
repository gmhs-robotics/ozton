//! Tracked-drive commands that borrow only the drivetrain.
use std::time::Duration;

use ozton_drivetrain::{
    Drive, Drivetrain, Footprint,
    model::{Differential, DriveEncoders, WheelSpeedError},
};
use ozton_localization::{
    Fault, Feedback,
    differential::{DifferentialLocalization, update_localization},
};
use ozton_math::{
    geometry::{DVec2, Pose},
    uom::si::f64::Velocity,
};
use ozton_runtime::drive_assist::{AssistedDrivetrain, DriveAssist};
use ozton_tracking::{Tracking, sampling::LocalizationMode};

type DriveOutputError = <Differential as Drive>::Error;

type TrackedDrive = Drivetrain<Differential, Tracking, DifferentialLocalization>;
type AssistedTrackedDrive<'a, A> =
    AssistedDrivetrain<'a, Differential, Tracking, DifferentialLocalization, A>;
#[derive(Clone, Copy, Debug)]
pub enum DriveMotionFault {
    Tracking(Fault),
    Output(DriveOutputError),
    Brake(DriveOutputError),
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

/// Wheel-speed output, live tracking, and field geometry for pose seeking.
pub trait PoseDrive: DriveEncoders {
    type Error;

    fn pose(&self) -> Pose;
    fn footprint(&self) -> Option<Footprint>;
    fn rectangular_half_extents(&self) -> Option<DVec2>;
    fn pose_trusted(&self) -> bool;
    fn sample(&mut self) -> Result<Option<Feedback>, Self::Error>;
    fn drive_wheels(&mut self, speeds: [Velocity; 2]) -> Result<(), Self::Error>;
    fn brake(&mut self) -> Result<(), Self::Error>;
    fn request_distance_reset(&mut self);
    fn cancel_distance_reset(&mut self);

    fn log_step(&mut self, _elapsed: Duration, _dt: Duration) {}
}

impl PoseDrive for TrackedDrive {
    type Error = DriveMotionFault;

    fn pose(&self) -> Pose {
        self.localization.pose()
    }

    fn footprint(&self) -> Option<Footprint> {
        self.footprint()
    }

    fn rectangular_half_extents(&self) -> Option<DVec2> {
        self.localization.rectangular_half_extents()
    }

    fn sample(&mut self) -> Result<Option<Feedback>, Self::Error> {
        self.localization
            .set_competition_mode(LocalizationMode::Autonomous);
        update_localization(&mut self.localization, &mut self.tracking)
            .map_err(DriveMotionFault::Tracking)
    }

    fn drive_wheels(&mut self, speeds: [Velocity; 2]) -> Result<(), Self::Error> {
        self.drive_wheel_speeds(speeds).map_err(Into::into)
    }

    fn brake(&mut self) -> Result<(), Self::Error> {
        self.stop_now().map_err(DriveMotionFault::Brake)
    }

    fn pose_trusted(&self) -> bool {
        self.localization.pose_trusted()
    }

    #[cfg(feature = "logging")]
    fn log_step(&mut self, elapsed: Duration, dt: Duration) {
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

impl<A: DriveAssist<Differential, Tracking, DifferentialLocalization>> PoseDrive
    for AssistedTrackedDrive<'_, A>
{
    type Error = DriveMotionFault;

    fn pose(&self) -> Pose {
        self.drivetrain().pose()
    }

    fn footprint(&self) -> Option<Footprint> {
        self.drivetrain().footprint()
    }

    fn rectangular_half_extents(&self) -> Option<DVec2> {
        self.drivetrain().rectangular_half_extents()
    }

    fn sample(&mut self) -> Result<Option<Feedback>, Self::Error> {
        self.drivetrain_mut().sample()
    }

    fn drive_wheels(&mut self, speeds: [Velocity; 2]) -> Result<(), Self::Error> {
        self.drive_wheel_speeds(speeds).map_err(Into::into)
    }

    fn brake(&mut self) -> Result<(), Self::Error> {
        self.stop_now().map_err(DriveMotionFault::Brake)
    }

    fn pose_trusted(&self) -> bool {
        self.drivetrain().pose_trusted()
    }

    #[cfg(feature = "logging")]
    fn log_step(&mut self, elapsed: Duration, dt: Duration) {
        self.drivetrain_mut().log_step(elapsed, dt);
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
    A: DriveAssist<Differential, Tracking, DifferentialLocalization>,
{
    fn reset(&mut self, starting_pose: Pose) -> Result<(), DriveMotionFault> {
        self.stop_now().map_err(DriveMotionFault::Brake)?;
        self.drivetrain_mut().reset(starting_pose)
    }
}
