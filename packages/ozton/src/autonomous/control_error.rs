//! A result type shared by controller, mechanism, and motion callbacks.

use core::{error::Error, fmt};

use ozton_control::mechanism::MechanismError;
use ozton_drivetrain::model::WheelSpeedError;
use ozton_motion::seeking::pose::{DriveMotionFault, PoseMotionError};
use vexide::controller::ControllerError;

use super::CommandError;

pub type ControlResult<T = ()> = Result<T, ControlError>;

#[derive(Debug)]
pub enum ControlError {
    Controller(ControllerError),
    Mechanism(MechanismError),
    Drive(WheelSpeedError),
    Motion(CommandError<DriveMotionFault>),
    PoseMotion(PoseMotionError<DriveMotionFault>),
}

impl fmt::Display for ControlError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Controller(error) => write!(f, "Controller unavailable: {error}"),
            Self::Mechanism(error) => error.fmt(f),
            Self::Drive(error) => write!(f, "Drive fault: {error}"),
            Self::Motion(error) => write!(f, "Motion fault: {error}"),
            Self::PoseMotion(error) => write!(f, "Motion fault: {error}"),
        }
    }
}

impl Error for ControlError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Controller(error) => Some(error),
            Self::Mechanism(error) => Some(error),
            Self::Drive(error) => Some(error),
            Self::Motion(error) => Some(error),
            Self::PoseMotion(error) => Some(error),
        }
    }
}

impl From<ControllerError> for ControlError {
    fn from(error: ControllerError) -> Self {
        Self::Controller(error)
    }
}

impl From<MechanismError> for ControlError {
    fn from(error: MechanismError) -> Self {
        Self::Mechanism(error)
    }
}

impl From<WheelSpeedError> for ControlError {
    fn from(error: WheelSpeedError) -> Self {
        Self::Drive(error)
    }
}

impl From<DriveMotionFault> for ControlError {
    fn from(error: DriveMotionFault) -> Self {
        Self::Motion(CommandError::Robot(error))
    }
}

impl From<CommandError<DriveMotionFault>> for ControlError {
    fn from(error: CommandError<DriveMotionFault>) -> Self {
        Self::Motion(error)
    }
}

impl From<PoseMotionError<DriveMotionFault>> for ControlError {
    fn from(error: PoseMotionError<DriveMotionFault>) -> Self {
        Self::PoseMotion(error)
    }
}
