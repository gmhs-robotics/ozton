//! Field geometry and pose estimation using tracking observations.

pub mod field;
pub mod localizer;

pub use localizer::*;

pub mod differential;
mod feedback;
pub use feedback::{Feedback, RangeFeedback};

/// Field-coordinate position covariance in square metres.
pub type PositionCovariance = glam::DMat2;
