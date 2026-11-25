//! Shared hardware and competition utilities.

pub mod clock;
pub mod competition;
pub mod control_error;
pub use ozton_drivetrain::device;
pub mod diagnostics;
pub mod drive_assist;
pub mod mechanism;
pub mod motor;
mod startup;
pub mod task;
pub use ozton_diagnostics::{log_event, log_row, profile};
pub use ozton_ui::{menu, selector};
pub use startup::{Application, run};

pub use competition::{OztonCompete, Runtime};

pub mod localization;
pub use localization::LocalizationSchedule;
