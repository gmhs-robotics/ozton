//! Motion control algorithms.

mod curvature;

pub mod assists;
pub mod basic;
pub mod commands;
pub mod pursuit;
pub mod seeking;

pub use basic::Basic;
use commands::routine_settings::{
    AUTON_POSITION_TOLERANCE, AUTON_SETTLE_TIME, AUTON_SPEED_TOLERANCE,
};
pub use commands::{DriveTuning, PoseSeeking};
pub use curvature::CurvatureDrive;
use ozton_control::{Tolerances, axis::Linear};
pub use pursuit::PurePursuit;
pub use seeking::Seeking;

const LINEAR_SETTLING: Tolerances<Linear> = Tolerances::new()
    .error(AUTON_POSITION_TOLERANCE)
    .velocity(AUTON_SPEED_TOLERANCE)
    .duration(AUTON_SETTLE_TIME);

/// Normal completion status for feedback-driven motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionOutcome {
    Settled,
    TimedOut,
}

#[derive(Debug, PartialEq, Eq)]
pub enum MotionError<E> {
    InvalidTarget,
    InvalidFeedback,
    InvalidOutput,
    InvalidPath,
    Output(E),
    Stop(E),
    OutputAndStop { output: E, stop: E },
}

fn finish_motion<E>(
    stopped: Result<(), E>,
    outcome: MotionOutcome,
) -> Result<MotionOutcome, MotionError<E>> {
    stopped.map(|()| outcome).map_err(MotionError::Stop)
}

fn motion_output_error<E>(output: E, stopped: Result<(), E>) -> MotionError<E> {
    match stopped {
        Ok(()) => MotionError::Output(output),
        Err(stop) => MotionError::OutputAndStop { output, stop },
    }
}
