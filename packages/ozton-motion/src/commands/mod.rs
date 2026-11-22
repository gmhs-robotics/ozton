//! Programmed autonomous commands, tracking and mechanism coordination.

mod calibration;
pub mod chassis;
mod command;
mod controls;
mod convenience;
mod drivetrain;
mod robot_motion;
mod routine_error;
pub mod routine_settings;
mod runner;
mod sequence;
mod setup;
mod stage;

pub use calibration::{Calibration, CalibrationSession, CalibrationTrial};
pub use command::{Alignment, Command, Contact, DriveOptions, PathMarker, RangeFacing};
pub use controls::{MotionControls, ScoringControls};
pub use drivetrain::{DriveMotionFault, DrivetrainMotion, GoalReset};
pub use ozton_localization::{Feedback, RangeFeedback};
pub use ozton_math::path::follower::Follower;
pub use robot_motion::{RobotMotion, TravelDirection};
pub use routine_error::RoutineError;
pub use runner::{CommandError, MotionPlatform, Runner};
pub use sequence::MechanismRobot;
pub use setup::{
    ChassisDriverCommand, ConfiguredChassis, DifferentialConfig, SetupError, TrackingSetup,
};
pub use stage::TimedMotion;

pub use crate::assists::{
    AntiTipAssist, AssistedMotion, HeadingHoldAssist, MotorBalanceAssist, TiltAxis,
};
