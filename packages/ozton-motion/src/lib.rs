//! Motion control algorithms.

mod curvature;

pub mod basic;
pub mod commands;
pub mod pursuit;
pub mod seeking;

pub use basic::Basic;
pub use curvature::CurvatureDrive;
pub use pursuit::PurePursuit;
pub use seeking::Seeking;

/// Normal completion status for feedback-driven motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionOutcome {
    Settled,
    TimedOut,
}

#[derive(Debug, PartialEq, Eq)]
pub enum MotionError<E> {
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
