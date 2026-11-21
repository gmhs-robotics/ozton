//! Programmed autonomous commands, tracking and mechanism coordination.

mod calibration;
pub mod chassis;
mod command;
mod competition;
mod controls;
mod convenience;
mod localization;
mod robot;
mod robot_motion;
mod route;
mod routine_error;
pub mod routine_settings;
mod runner;
mod sequence;
mod setup;

pub use calibration::{Calibration, CalibrationTrial};
pub use command::{
    Alignment, Command, Contact, DriveOptions, Feedback, PathMarker, RangeFacing, RangeFeedback,
};
pub use competition::{ProgrammedRobot, compete};
pub use controls::{AutonomousControls, ScoringControls};
pub use localization::DifferentialLocalization;
pub use ozton_math::path::follower::Follower;
pub use robot::{Robot, RobotBuilder, StockTick};
pub use robot_motion::{RobotMotion, TravelDirection};
pub use route::{
    AlignTarget, Auton, RegistrationError, RegistrationProblem, Route, RouteResult, compete_routes,
    compete_with_calibration,
};
pub use routine_error::RoutineError;
pub use runner::{AutonError, AutonRobot, Runner};
pub use sequence::{MechanismActions, MechanismRobot};
pub use setup::{ConfiguredChassis, DifferentialConfig, SetupError, TrackingSetup};
