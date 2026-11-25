//! Motion control algorithms.

pub mod arcade;
pub mod curvature;
pub mod holonomic;
pub mod tank;

pub mod assists;
mod basic;
pub mod config;
pub use config::{
    ApproachConfig, ClearanceConfig, MotionConfig, MotionLimits, MotionTiming, PathConfig,
    PathGuidance, RateFilters, SettlingConfig,
};
mod pursuit;
pub mod seeking;
#[cfg(not(target_os = "vexos"))]
pub mod simulation;

pub use arcade::Arcade;
pub use basic::differential::DriveTuning;
pub use curvature::CurvatureDrive;
pub use holonomic::Holonomic;
pub use seeking::Seeking;
pub use tank::Tank;
