//! Drive encoder observation interface.

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
mod wheels;

pub use wheels::{DriveConfigError, WheelFeedback, WheelGearing, WheelLoad, Wheels, positive};
