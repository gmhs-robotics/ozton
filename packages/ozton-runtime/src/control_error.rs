//! A result type shared by controller, mechanism, and motion callbacks.

use core::{error::Error, fmt};

use ozton_drivetrain::{DriveMotionFault, model::WheelSpeedError};
use ozton_motion::seeking::PoseMotionError;
use vexide::controller::ControllerError;

use crate::mechanism::MechanismError;

pub type ControlResult<T = ()> = Result<T, ControlError>;

#[derive(Debug)]
pub enum ControlError {
    Controller(ControllerError),
    Mechanism(MechanismError),
    Drive(WheelSpeedError),
    PoseMotion(PoseMotionError<DriveMotionFault>),
}

impl fmt::Display for ControlError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Controller(error) => write!(f, "Controller unavailable: {error}"),
            Self::Mechanism(error) => error.fmt(f),
            Self::Drive(error) => write!(f, "Drive fault: {error}"),
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
        Self::PoseMotion(PoseMotionError::Drive(error))
    }
}

impl From<PoseMotionError<DriveMotionFault>> for ControlError {
    fn from(error: PoseMotionError<DriveMotionFault>) -> Self {
        Self::PoseMotion(error)
    }
}
