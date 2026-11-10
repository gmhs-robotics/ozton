//! Programmed autonomous commands, tracking and mechanism coordination.

mod calibration;
pub mod chassis;
mod command;
mod competition;
mod robot;
mod route;
mod runner;
mod setup;

pub use calibration::{Calibration, CalibrationTrial};
pub use command::{Alignment, Command, Contact, DriveOptions, Feedback, PathMarker, RangeFeedback};
pub use competition::{ProgrammedRobot, compete};
pub use robot::{Mechanisms, Robot};
pub use route::{
    AlignTarget, Auton, RegistrationError, RegistrationProblem, Route, RouteResult, compete_routes,
    compete_with_calibration,
};
pub use runner::{AutonError, AutonRobot, Runner};
pub use setup::{ConfiguredChassis, DifferentialConfig, RangeId, SetupError, TrackingSetup};
