//! Tracked-drive commands that borrow only the drivetrain.
use std::time::Duration;

use ozton_drivetrain::{
    Drive, Drivetrain, Footprint,
    model::{Differential, DriveEncoders, WheelSpeedError},
};
use ozton_localization::{Fault, Feedback, differential::DifferentialLocalization};
use ozton_math::{
    geometry::{DVec2, Pose},
    uom::si::f64::Velocity,
};
use ozton_runtime::drive_assist::{AssistedDrivetrain, DriveAssist};
use ozton_tracking::{Tracking, TrackingSources};

type DriveOutputError = <Differential as Drive>::Error;

type TrackedDrive<S> = Drivetrain<Differential, Tracking<S>, DifferentialLocalization>;
type AssistedTrackedDrive<'a, S, A> =
    AssistedDrivetrain<'a, Differential, Tracking<S>, DifferentialLocalization, A>;
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
    fn pose_feedback(&self) -> Result<Option<Feedback>, Self::Error>;
    fn drive_wheels(&mut self, speeds: [Velocity; 2]) -> Result<(), Self::Error>;
    fn brake(&mut self) -> Result<(), Self::Error>;
    fn request_distance_reset(&mut self);
    fn cancel_distance_reset(&mut self);

    fn log_step(&mut self, _elapsed: Duration, _dt: Duration) {}
}

impl<S: TrackingSources> PoseDrive for TrackedDrive<S> {
    type Error = DriveMotionFault;

    fn pose(&self) -> Pose {
        self.localization().pose()
    }

    fn footprint(&self) -> Option<Footprint> {
        self.footprint()
    }

    fn rectangular_half_extents(&self) -> Option<DVec2> {
        self.localization().rectangular_half_extents()
    }

    fn pose_feedback(&self) -> Result<Option<Feedback>, Self::Error> {
        self.localization()
            .feedback()
            .map_err(DriveMotionFault::Tracking)
    }

    fn drive_wheels(&mut self, speeds: [Velocity; 2]) -> Result<(), Self::Error> {
        self.drive_wheel_speeds(speeds).map_err(Into::into)
    }

    fn brake(&mut self) -> Result<(), Self::Error> {
        self.stop_now().map_err(DriveMotionFault::Brake)
    }

    fn pose_trusted(&self) -> bool {
        self.localization().pose_trusted()
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
        self.localization_mut().request_distance_reset();
    }

    fn cancel_distance_reset(&mut self) {
        self.localization_mut().cancel_distance_reset();
    }
}

impl<S, A> PoseDrive for AssistedTrackedDrive<'_, S, A>
where
    S: TrackingSources,
    A: DriveAssist<Differential, Tracking<S>, DifferentialLocalization>,
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

    fn pose_feedback(&self) -> Result<Option<Feedback>, Self::Error> {
        self.drivetrain().pose_feedback()
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
