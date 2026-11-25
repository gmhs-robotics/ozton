use core::{error::Error, fmt};

/// Failure to finish a pose-seeking motion or stop its output.
#[derive(Debug)]
pub enum PoseMotionError<E> {
    Configuration,
    Cancelled,
    StageTimeout,
    ControlGap,
    InvalidFeedback,
    InsufficientClearance,
    PositionUnsettled,
    UntrustedPose,
    NoProgress,
    Mechanism(Box<dyn Error>),
    Drive(E),
    StopFailed(E),
    StopFailedAfter { cause: Box<Self>, stop: E },
}

impl<E> PoseMotionError<E> {
    pub(super) const fn reason(&self) -> &'static str {
        match self {
            Self::Configuration => "configuration",
            Self::Cancelled => "cancelled",
            Self::StageTimeout => "command_timeout",
            Self::ControlGap => "control_gap",
            Self::InvalidFeedback => "invalid_feedback",
            Self::InsufficientClearance => "insufficient_clearance",
            Self::PositionUnsettled => "position_unsettled",
            Self::UntrustedPose => "untrusted_pose",
            Self::NoProgress => "no_progress",
            Self::Mechanism(_) => "mechanism",
            Self::Drive(_) => "robot",
            Self::StopFailed(_) | Self::StopFailedAfter { .. } => "stop_failed",
        }
    }
}

impl<E: fmt::Display> fmt::Display for PoseMotionError<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let description = match self {
            Self::Configuration => "Invalid pose-seeking configuration",
            Self::Cancelled => "Pose seeking cancelled",
            Self::StageTimeout => "Pose-seeking stage timed out",
            Self::ControlGap => "Pose-seeking control tick delayed",
            Self::InvalidFeedback => "Invalid pose-seeking feedback",
            Self::InsufficientClearance => "Insufficient clearance for motion",
            Self::PositionUnsettled => "Position shifted outside tolerance during the final turn",
            Self::UntrustedPose => "Untrusted field pose",
            Self::NoProgress => "Pose seeking made no progress",
            Self::Mechanism(error) => return error.fmt(f),
            Self::Drive(error) => return error.fmt(f),
            Self::StopFailed(error) => return write!(f, "Motion stop failed: {error}"),
            Self::StopFailedAfter { cause, stop } => {
                return write!(f, "{cause}. Motion stop failed: {stop}");
            }
        };
        f.write_str(description)
    }
}

impl<E: Error + 'static> Error for PoseMotionError<E> {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Drive(error) | Self::StopFailed(error) => Some(error),
            Self::Mechanism(error) => Some(error.as_ref()),
            Self::StopFailedAfter { cause, .. } => Some(cause.as_ref()),
            _ => None,
        }
    }
}
