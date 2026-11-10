//! Shared hardware and competition utilities.

pub mod clock;
pub mod motor;
pub mod selector;

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
