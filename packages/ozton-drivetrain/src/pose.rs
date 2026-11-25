//! Hardware-independent contract for a localized drivetrain.
use std::{
    future::Future,
    time::{Duration, Instant},
};

use ozton_localization::Feedback;
use ozton_math::{
    geometry::{DVec2, Pose},
    path::clearance::{Footprint, MAX_OBSTACLES, Obstacles},
    uom::si::f64::Velocity,
};

use crate::model::DriveEncoders;

#[cfg(feature = "vex")]
mod vex;
#[cfg(feature = "vex")]
pub use vex::DriveMotionFault;

/// Wheel-speed output, live tracking, and field geometry for pose seeking.
pub trait PoseDrive: DriveEncoders {
    type Error;

    fn pose(&self) -> Pose;
    fn footprint(&self) -> Option<Footprint>;
    fn rectangular_half_extents(&self) -> Option<DVec2>;
    fn motion_obstacles(&self) -> Obstacles {
        [None; MAX_OBSTACLES]
    }
    fn movable_obstacles(&self) -> Obstacles {
        [None; MAX_OBSTACLES]
    }
    fn pose_trusted(&self) -> bool;
    fn pose_feedback(&self) -> Result<Option<Feedback>, Self::Error>;
    /// Estimated XY covariance in field coordinates, in square metres.
    fn position_covariance(&self) -> Option<glam::DMat2> {
        None
    }
    /// Rate standard deviations in right, forward, yaw order (SI).
    fn velocity_uncertainty(&self) -> [f64; 3] {
        [0.0; 3]
    }
    fn drive_wheels(&mut self, speeds: [Velocity; 2]) -> Result<(), Self::Error>;
    fn brake(&mut self) -> Result<(), Self::Error>;
    fn request_distance_reset(&mut self);
    fn cancel_distance_reset(&mut self);

    /// Yield until the monotonic deadline using the platform executor.
    fn wait_until(&mut self, deadline: Instant) -> impl Future<Output = ()>;

    /// False when a competition or application cancellation forbids output.
    fn motion_active(&self) -> bool;

    fn log_step(&mut self, _elapsed: Duration, _dt: Duration) {}
}
