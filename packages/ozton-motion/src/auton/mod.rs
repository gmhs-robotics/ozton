//! Programmed autonomous commands, tracking and mechanism coordination.

mod calibration;
pub mod chassis;
mod command;
mod controls;
mod convenience;
mod drivetrain;
mod localization;
mod robot_motion;
mod routine_error;
pub mod routine_settings;
mod runner;
mod sequence;
mod setup;
mod stage;

pub use calibration::{Calibration, CalibrationSession, CalibrationTrial};
pub use command::{
    Alignment, Command, Contact, DriveOptions, Feedback, PathMarker, RangeFacing, RangeFeedback,
};
pub use controls::{AutonomousControls, ScoringControls};
pub use drivetrain::{DriveMotionFault, DrivetrainMotion, GoalReset};
pub use localization::DifferentialLocalization;
mod tracking_builder;
pub use ozton_math::path::follower::Follower;
pub use robot_motion::{RobotMotion, TravelDirection};
pub use routine_error::RoutineError;
pub use runner::{AutonError, AutonRobot, Runner};
pub use sequence::MechanismRobot;
pub use setup::{
    ChassisDriverCommand, ConfiguredChassis, DifferentialConfig, SetupError, TrackingSetup,
};
pub use stage::Auton;
pub use tracking_builder::{DriveEncoderUse, DsrUse, ImuUse, TrackingBuildError, TrackingBuilder};
