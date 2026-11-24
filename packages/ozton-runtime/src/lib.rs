//! Shared hardware and competition utilities.

pub mod clock;
pub mod competition;
pub mod device;
pub mod diagnostics;
pub mod drive_assist;
pub mod mechanism;
pub mod motor;
mod robot;
mod startup;
pub mod task;
pub use ozton_diagnostics::{log_event, log_row, profile};
pub use ozton_ui::{menu, selector};
pub use startup::{Application, run};

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

pub use competition::{OztonCompete, Runtime};
pub use robot::{CompetitionBot, Robot, RobotBuilder, RobotLayout};

pub mod localization;
pub use localization::LocalizationSchedule;
