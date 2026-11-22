//! Pose and relative range feedback for motion consumers.
use std::time::Duration;

use ozton_math::{
    geometry::Pose,
    uom::si::f64::{AngularVelocity, Length, Velocity},
};
use ozton_tracking::sources::distance::MAX_RANGES;

#[derive(Debug, Clone, Copy)]
pub struct RangeFeedback {
    pub distance: Length,
    pub timestamp: Duration,
}

/// One tick of pose at The Origin in X-right/Y-forward metres and CCW radians from +Y.
#[derive(Debug, Clone, Copy)]
pub struct Feedback {
    /// Snapshot time on the same monotonic clock as range packets.
    pub time: Duration,
    pub pose: Pose,
    pub forward_travel: Length,
    pub forward_velocity: Velocity,
    /// Positive toward robot +X (right), metres per second.
    pub lateral_velocity: Velocity,
    pub angular_velocity: AngularVelocity,
    pub ranges: [Option<RangeFeedback>; MAX_RANGES],
    pub reset_accepted: bool,
}
