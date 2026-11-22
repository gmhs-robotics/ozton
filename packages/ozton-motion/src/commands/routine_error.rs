//! Allocation-free errors for premade autonomous commands.
use core::{error::Error, fmt};

use vexide::smart::PortError;

/// Identifies a command failure without requiring callers to inspect log text.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RoutineError<M> {
    Tracking(ozton_localization::localizer::Fault),
    TrackingUnavailable,
    FieldPoseLost,
    InvalidField,
    InvalidSessionLimits,
    InvalidPose,
    InvalidPath,
    InvalidGoalReset,
    ContactFeedbackUnavailable,
    UnexpectedContact,
    OutsideGoalApproach,
    InvalidAlignmentSensor,
    MotionFault,
    InvalidMotionFeedback,
    PathStart,
    PathLost,
    NoProgress,
    Cancelled,
    DriverCancelled,
    DriverTimeout,
    RouteTimeout,
    CommandTimeout,
    ControlDelay,
    InvalidLiftLimits,
    InvalidPivotLimits,
    InvalidLiftTarget,
    InvalidPivotTarget,
    InvalidDistance,
    NegativeForwardDistance,
    NegativeBackwardDistance,
    MechanismFault(M),
    MechanismBrake(PortError),
    DriveOutput(PortError),
    DriveBrake(PortError),
}

impl<M> RoutineError<M> {
    /// Translate runner failures while retaining the cause of a failed stop in diagnostics.
    pub fn from_runner(error: super::CommandError<Self>) -> Self
    where
        M: fmt::Debug,
    {
        use super::CommandError as Motion;
        match error {
            Motion::Robot(error) | Motion::StopFailed(error) => error,
            Motion::StopFailedAfter { cause, stop } => {
                let cause = Self::from_runner(*cause);
                ozton_runtime::diagnostic_event!(
                    "auton_motion_failed,cause={cause:?},stop={stop:?}"
                );
                stop
            }
            Motion::Cancelled => Self::Cancelled,
            Motion::RouteTimeout => Self::RouteTimeout,
            Motion::CommandTimeout => Self::CommandTimeout,
            Motion::ControlGap => Self::ControlDelay,
            Motion::Configuration => Self::InvalidSessionLimits,
            Motion::InvalidFeedback => Self::InvalidMotionFeedback,
            Motion::InvalidPath => Self::InvalidPath,
            Motion::PathStart => Self::PathStart,
            Motion::PathLost => Self::PathLost,
            Motion::UntrustedPose => Self::FieldPoseLost,
            Motion::NoProgress => Self::NoProgress,
        }
    }
    /// Hardware feedback/output faults may recover on a subsequent safe tick.
    pub const fn recoverable(&self) -> bool {
        matches!(self, Self::Tracking(_) | Self::DriveOutput(_))
    }

    /// An optional command may skip a recoverable fault or a mechanism fault.
    pub const fn can_skip_failure(&self) -> bool {
        self.recoverable() || matches!(self, Self::MechanismFault(_))
    }

    /// Stable, allocation-free text for the Brain's existing status screen.
    pub const fn status(&self) -> &'static str {
        match self {
            Self::Tracking(_) => "STOP: tracking feedback unavailable",
            Self::TrackingUnavailable => "STOP: tracking not configured",
            Self::FieldPoseLost => "STOP: fused field pose lost",
            Self::InvalidField => "STOP: invalid field configuration",
            Self::InvalidSessionLimits => "STOP: invalid autonomous deadlines",
            Self::InvalidPose => "STOP: invalid field pose",
            Self::InvalidPath => "STOP: invalid path export",
            Self::InvalidGoalReset => "STOP: invalid goal reset settings",
            Self::ContactFeedbackUnavailable => "STOP: contact motor feedback unavailable",
            Self::UnexpectedContact => "STOP: contact outside reset window",
            Self::OutsideGoalApproach => "STOP: outside goal approach",
            Self::InvalidAlignmentSensor => "STOP: alignment requires front/rear sensor",
            Self::MotionFault => "STOP: autonomous motion failure",
            Self::InvalidMotionFeedback => "STOP: invalid motion feedback",
            Self::PathStart => "STOP: outside path start",
            Self::PathLost => "STOP: path lost",
            Self::NoProgress => "STOP: no autonomous progress",
            Self::Cancelled => "Autonomous cancelled",
            Self::DriverCancelled => "Driver routine cancelled",
            Self::DriverTimeout => "Driver routine timed out",
            Self::RouteTimeout => "STOP: autonomous total timeout",
            Self::CommandTimeout => "STOP: autonomous command timeout",
            Self::ControlDelay => "STOP: autonomous control delay",
            Self::InvalidLiftLimits => "STOP: invalid lift limits",
            Self::InvalidPivotLimits => "STOP: invalid pivot limits",
            Self::InvalidLiftTarget => "STOP: invalid lift target",
            Self::InvalidPivotTarget => "STOP: invalid pivot target",
            Self::InvalidDistance => "STOP: invalid drive distance",
            Self::NegativeForwardDistance => "STOP: forward distance must be positive",
            Self::NegativeBackwardDistance => "STOP: backward distance must be positive",
            Self::MechanismFault(_) => "STOP: autonomous mechanism fault",
            Self::MechanismBrake(_) => "STOP: mechanism brake fault",
            Self::DriveOutput(_) => "STOP: autonomous drive output fault",
            Self::DriveBrake(_) => "STOP: drive brake fault",
        }
    }
}

impl<M: fmt::Debug> From<super::CommandError<Self>> for RoutineError<M> {
    fn from(error: super::CommandError<Self>) -> Self {
        Self::from_runner(error)
    }
}

impl<M> From<ozton_runtime::competition::DriverWaitError> for RoutineError<M> {
    fn from(error: ozton_runtime::competition::DriverWaitError) -> Self {
        use ozton_runtime::competition::DriverWaitError;
        match error {
            DriverWaitError::Cancelled => Self::DriverCancelled,
            DriverWaitError::Timeout => Self::DriverTimeout,
        }
    }
}

impl<M> fmt::Display for RoutineError<M> {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.status())?;
        if let Self::DriveOutput(source) | Self::DriveBrake(source) | Self::MechanismBrake(source) =
            self
        {
            write!(formatter, ": {source}")?;
        }
        Ok(())
    }
}

impl<M: Error + 'static> Error for RoutineError<M> {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::MechanismFault(fault) => Some(fault),
            Self::DriveOutput(source) | Self::DriveBrake(source) | Self::MechanismBrake(source) => {
                Some(source)
            }
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type CommandError = RoutineError<core::convert::Infallible>;

    #[test]
    fn hardware_error_retains_source_while_ui_status_stays_stable() {
        let error = CommandError::DriveOutput(PortError::Disconnected { port: 5 });
        assert_eq!(error.status(), "STOP: autonomous drive output fault");
        assert!(error.source().is_some());
        assert!(error.to_string().contains("port 5"));
        assert!(CommandError::Cancelled.source().is_none());
    }
}
