//! Common failures for fallible mechanism commands.

use core::{error::Error, fmt};

use ozton_control::protection::ProgressFault;
use vexide::smart::PortError;

pub type MechanismResult<T = ()> = Result<T, MechanismError>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MechanismError {
    Port(PortError),
    Protection(ProgressFault),
    InvalidFeedback,
    InvalidTarget,
    Timeout,
    Cancelled,
    ControlGap,
}

impl fmt::Display for MechanismError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Port(error) => write!(f, "Mechanism port fault: {error}"),
            Self::Protection(error) => write!(f, "Mechanism protection: {error:?}"),
            Self::InvalidFeedback => f.write_str("Invalid mechanism feedback"),
            Self::InvalidTarget => f.write_str("Invalid mechanism target"),
            Self::Timeout => f.write_str("Mechanism command timed out"),
            Self::Cancelled => f.write_str("Mechanism command cancelled"),
            Self::ControlGap => f.write_str("Mechanism control tick delayed"),
        }
    }
}

impl Error for MechanismError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Port(error) => Some(error),
            _ => None,
        }
    }
}

impl From<PortError> for MechanismError {
    fn from(error: PortError) -> Self {
        Self::Port(error)
    }
}

impl From<ProgressFault> for MechanismError {
    fn from(error: ProgressFault) -> Self {
        Self::Protection(error)
    }
}
