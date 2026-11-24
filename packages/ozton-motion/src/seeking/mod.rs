//! Physical-velocity feedback seeking.

mod pose;
pub use pose::{DriveMotionFault, PathActions, PoseDrive, PoseMotionError, StoppingGovernor, TravelDirection};

/// Reusable physical motion template for a calibrated drivetrain.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Seeking {
    config: crate::MotionConfig,
    travel: Option<TravelDirection>,
}

impl Seeking {
    /// Choose a direction rather than letting seeking select one.
    #[must_use]
    pub const fn with_travel(mut self, travel: TravelDirection) -> Self {
        self.travel = Some(travel);
        self
    }

    #[must_use]
    pub const fn without_travel(mut self) -> Self {
        self.travel = None;
        self
    }
}
