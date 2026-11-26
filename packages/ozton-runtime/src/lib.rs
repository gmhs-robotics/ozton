//! Shared hardware and competition utilities.

pub mod clock;
pub mod competition;
pub mod control_error;
pub use ozton_drivetrain::device;
pub mod diagnostics;
pub mod mechanism;
pub mod motor;
mod startup;
pub mod task;
pub use competition::{OztonCompete, Runtime};
pub use ozton_diagnostics::{log_event, log_row, profile};
pub use ozton_ui::{menu, selector};
pub use startup::{
    Application, BuildMetadata, SourceRevision, Startup, StartupError, run, run_with_metadata,
};

pub mod localization;
pub use localization::LocalizationSchedule;

/// Retries an initial reference until established. Reconnection preserves the zero.
#[derive(Debug, Default)]
pub struct StartupReference {
    initialized: bool,
}

impl StartupReference {
    pub fn initialize<E>(&mut self, initialize: impl FnOnce() -> Result<(), E>) -> Result<(), E> {
        if !self.initialized {
            initialize()?;
            self.initialized = true;
        }
        Ok(())
    }

    pub fn is_initialized(&self) -> bool {
        self.initialized
    }
}
