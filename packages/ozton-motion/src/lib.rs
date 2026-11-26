//! Motion control algorithms.

mod basic;
pub mod config;
pub use config::{
    ApproachConfig, MotionConfig, MotionLimits, MotionTiming, PathConfig, PathGuidance,
    RateFilters, SettlingConfig, TerminalGuidance,
};
mod pursuit;
mod seeking;
#[cfg(not(target_os = "vexos"))]
pub mod simulation;

pub use basic::differential::DriveTuning;
pub use seeking::*;
