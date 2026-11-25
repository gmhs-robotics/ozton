//! Physical-velocity feedback seeking.

mod chassis;
mod pose;
pub use chassis::Chassis;
pub use pose::{
    BrakeMode, MotionExecution, PathActions, PoseMotionError, RangeResetDrive, TravelDirection,
    WallContact, WheelActuation, WheelCommand,
};

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
