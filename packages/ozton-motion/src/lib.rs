//! Motion control algorithms.

pub mod arcade;
pub mod curvature;
pub mod holonomic;
pub mod tank;

#[cfg(feature = "vex")]
pub mod assists;
pub mod basic;
pub mod config;
pub use config::{
    ApproachConfig, ClearanceConfig, MotionConfig, MotionLimits, MotionTiming, PathConfig,
    RateFilters, SettlingConfig,
};
pub mod pursuit;
#[cfg(feature = "vex")]
pub mod seeking;

pub use arcade::Arcade;
pub use basic::differential::DriveTuning;
pub use curvature::CurvatureDrive;
pub use holonomic::Holonomic;
pub use tank::Tank;
#[cfg(feature = "vex")]
pub use seeking::Seeking;

pub mod ramsete;
