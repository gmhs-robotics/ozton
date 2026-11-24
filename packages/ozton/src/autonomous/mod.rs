//! Programmed autonomous commands, tracking and mechanism coordination.

mod calibration;
pub mod chassis;
mod command;
mod command_error;
mod control_error;
mod controls;
mod convenience;
mod robot_motion;
mod routine_error;
pub mod routine_settings;
mod runner;
mod sequence;
mod setup;
mod stage;

pub use calibration::{Calibration, CalibrationSession, CalibrationTrial};
pub use command::{Alignment, Command, Contact, PathMarker, RangeFacing};
pub use command_error::CommandError;
pub use control_error::{ControlError, ControlResult};
pub use controls::{MotionControls, ScoringControls};
pub use ozton_localization::{Feedback, RangeFeedback};
pub use ozton_motion::{
    assists::{AntiTipAssist, HeadingHoldAssist, MotorBalanceAssist, TiltAxis},
    basic::differential::{DriveOptions, DriveTuning},
    pursuit::profiled::Follower,
    seeking::pose::TravelDirection,
};
pub use robot_motion::RobotMotion;
pub use routine_error::RoutineError;
pub use runner::{MotionPlatform, Runner};
pub use sequence::MechanismRobot;
pub use setup::{
    ChassisDriverCommand, ConfiguredChassis, DifferentialConfig, SetupError, TrackingSetup,
};
pub use stage::TimedMotion;
