//! Programmed autonomous commands, tracking and mechanism coordination.

mod calibration;
pub mod chassis;
mod command;
mod competition;
mod controls;
mod convenience;
mod robot;
mod route;
mod runner;
mod sequence;
mod setup;

pub use calibration::{Calibration, CalibrationTrial};
pub use command::{
    Alignment, Command, Contact, DriveOptions, Feedback, PathMarker, RangeFacing, RangeFeedback,
};
pub use competition::{ProgrammedRobot, compete};
pub use controls::{AutonomousControls, ScoringControls};
pub use robot::{Robot, StockTick};
pub use route::{
    AlignTarget, Auton, RegistrationError, RegistrationProblem, Route, RouteResult, compete_routes,
    compete_with_calibration,
};
pub use runner::{AutonError, AutonRobot, Runner};
pub use sequence::{MechanismActions, MechanismRobot};
pub use setup::{ConfiguredChassis, DifferentialConfig, RangeId, SetupError, TrackingSetup};
