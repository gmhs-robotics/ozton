//! Sources that measure relative travel from wheels.

/// Calibrated cumulative drive-wheel travel in left, right order, positive forward.
pub trait DriveEncoders {
    /// Return the drivetrain's surveyed track width when it is configured.
    fn track_width(&self) -> Option<ozton_math::uom::si::f64::Length> {
        None
    }

    /// Sample both sides once. An unavailable side returns `None`.
    fn tracking_positions(&mut self) -> [Option<ozton_math::uom::si::f64::Length>; 2];
}

pub mod consensus;
pub mod differential;
mod gearing;
mod motors;
pub use motors::SharedMotors;
pub mod holonomic;
mod projection;
pub mod wheel;

pub use differential::DifferentialTracking;
pub use gearing::{DriveConfigError, WheelFeedback, WheelGearing, Wheels, positive};
