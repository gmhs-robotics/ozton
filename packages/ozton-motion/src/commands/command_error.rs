//! Detailed failures from a motion command.

use core::{error::Error, fmt};

#[derive(Debug)]
pub enum CommandError<E> {
    Configuration,
    Cancelled,
    RouteTimeout,
    CommandTimeout,
    ControlGap,
    InvalidFeedback,
    InvalidPath,
    InsufficientClearance,
    PathStart,
    PathLost,
    UntrustedPose,
    NoProgress,
    Robot(E),
    /// An output could not be stopped. Even optional commands cannot recover from this.
    StopFailed(E),
    StopFailedAfter {
        cause: Box<Self>,
        stop: E,
    },
}

impl<E: fmt::Display> fmt::Display for CommandError<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let description = match self {
            Self::Configuration => "Invalid motion configuration",
            Self::Cancelled => "Motion cancelled",
            Self::RouteTimeout => "Route timed out",
            Self::CommandTimeout => "Motion command timed out",
            Self::ControlGap => "Motion control tick delayed",
            Self::InvalidFeedback => "Invalid motion feedback",
            Self::InvalidPath => "Invalid path",
            Self::InsufficientClearance => "Insufficient clearance for motion",
            Self::PathStart => "Outside path start",
            Self::PathLost => "Path lost",
            Self::UntrustedPose => "Untrusted field pose",
            Self::NoProgress => "No motion progress",
            Self::Robot(error) => return error.fmt(f),
            Self::StopFailed(error) => return write!(f, "Motion stop failed: {error}"),
            Self::StopFailedAfter { cause, stop } => {
                return write!(f, "{cause}. Motion stop failed: {stop}");
            }
        };
        f.write_str(description)
    }
}

impl<E: Error + 'static> Error for CommandError<E> {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Robot(error) | Self::StopFailed(error) => Some(error),
            Self::StopFailedAfter { cause, .. } => Some(cause.as_ref()),
            _ => None,
        }
    }
}
