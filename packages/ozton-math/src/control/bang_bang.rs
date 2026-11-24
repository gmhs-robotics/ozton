use core::time::Duration;

use super::Feedback;

/// Bang-bang controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BangBang {
    magnitude: f64,
}

impl BangBang {
    /// Creates a new bang-bang controller with a given output magnitude.
    pub const fn new(magnitude: f64) -> Self {
        Self { magnitude }
    }

    /// Returns the controller's output magnitude.
    pub const fn magnitude(&self) -> f64 {
        self.magnitude
    }

    /// Sets the controller's output magnitude.
    pub const fn set_magnitude(&mut self, magnitude: f64) {
        self.magnitude = magnitude;
    }

    #[must_use]
    pub const fn with_magnitude(mut self, value: f64) -> Self {
        self.magnitude = value;
        self
    }
}

impl Feedback for BangBang {
    type State = f64;
    type Signal = f64;
    fn update(&mut self, measurement: f64, setpoint: f64, _dt: Duration) -> f64 {
        if measurement < setpoint {
            self.magnitude
        } else {
            0.0
        }
    }
}
