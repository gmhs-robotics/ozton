//! Sources that measure relative travel from wheels.

use ozton_math::uom::si::f64::{Length, Velocity};

/// Continuous group travel and velocity from all available motors.
#[derive(Clone, Copy, Debug)]
pub struct WheelFeedback {
    pub position: Length,
    pub velocity: Velocity,
}

/// Calibrated cumulative drive-wheel travel in left, right order, positive forward.
pub trait DriveEncoders {
    /// Return the drivetrain's surveyed track width when it is configured.
    fn track_width(&self) -> Option<Length> {
        None
    }

    /// Sample both sides once. An unavailable side returns `None`.
    fn tracking_positions(&mut self) -> [Option<Length>; 2];
}

pub mod consensus;
#[cfg(feature = "vex")]
pub mod differential;
#[cfg(feature = "vex")]
mod gearing;
#[cfg(feature = "vex")]
mod motors;
#[cfg(feature = "vex")]
pub use motors::SharedMotors;
#[cfg(all(feature = "vex", feature = "logging"))]
pub(crate) use motors::command_targets;
mod projection;
pub mod wheel;

#[cfg(feature = "vex")]
pub use differential::DifferentialTracking;
#[cfg(feature = "vex")]
pub use gearing::{DriveConfigError, WheelGearing, Wheels, positive};
