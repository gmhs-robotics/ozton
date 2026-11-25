//! Field geometry and pose estimation using tracking observations.

pub mod field;
pub mod localizer;

pub use localizer::*;

mod access;
mod feedback;
mod localization;
pub use access::LocalizationEstimate;
pub use feedback::{Feedback, RangeFeedback};
pub use localization::{
    DriveEncoderUse, Localization, LocalizationBuilder, OdometrySettings, RangeRequired,
};

/// Field-coordinate position covariance in square metres.
pub type PositionCovariance = glam::DMat2;
