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
    RateFilters, SettlingConfig,
};
mod pursuit;
pub mod seeking;

pub use arcade::Arcade;
pub use basic::differential::DriveTuning;
pub use curvature::CurvatureDrive;
pub use holonomic::Holonomic;
pub use seeking::Seeking;
pub use tank::Tank;
