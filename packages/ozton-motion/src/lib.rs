//! Motion control algorithms.

pub mod arcade;
pub mod curvature;
pub mod holonomic;
pub mod tank;

#[cfg(feature = "vex")]
pub mod assists;
pub mod basic;
pub mod config;
pub use config::MotionConfig;
pub mod pursuit;
#[cfg(feature = "vex")]
pub mod seeking;

pub use arcade::Arcade;
pub use basic::{Basic, differential::DriveTuning};
use defaults::{AUTON_POSITION_TOLERANCE, AUTON_SETTLE_TIME, AUTON_SPEED_TOLERANCE};
pub use holonomic::Holonomic;
#[cfg(feature = "vex")]
pub use seeking::pose::PoseSeeking;
pub use tank::Tank;
mod defaults;
pub use curvature::CurvatureDrive;
use ozton_control::{Tolerances, axis::Linear};
pub use pursuit::PurePursuit;
#[cfg(feature = "vex")]
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

#[cfg(feature = "vex")]
fn finish_motion<E>(
    stopped: Result<(), E>,
    outcome: MotionOutcome,
) -> Result<MotionOutcome, MotionError<E>> {
    stopped.map(|()| outcome).map_err(MotionError::Stop)
}

#[cfg(feature = "vex")]
fn motion_output_error<E>(output: E, stopped: Result<(), E>) -> MotionError<E> {
    match stopped {
        Ok(()) => MotionError::Output(output),
        Err(stop) => MotionError::OutputAndStop { output, stop },
    }
}

pub mod ramsete;
