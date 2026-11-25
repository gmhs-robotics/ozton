//! Hardware-independent path tracking laws.
mod boomerang;
mod ramsete;
pub use boomerang::Boomerang;
pub use ramsete::Ramsete;

use crate::{
    geometry::{Pose, pursuit_curvature},
    uom::si::f64::Velocity,
};

/// Coulter's geometric pure pursuit controller. The target is in the
/// direction-of-travel frame, with +Y forward and +X right.
#[derive(Debug, Clone, Copy, Default)]
pub struct PurePursuit;

impl PurePursuit {
    pub const fn new() -> Self {
        Self
    }

    /// Return forward velocity and yaw rate in SI units.
    pub fn twist(
        self,
        pose: Pose,
        target: glam::DVec2,
        speed: Velocity,
        reverse: bool,
    ) -> (f64, f64) {
        let facing = crate::qty!((if reverse { core::f64::consts::PI } else { 0.0 }) rad);
        let travel = Pose {
            heading: pose.heading + facing,
            ..pose
        };
        let local = travel.to_local(target);
        let v = if reverse { -speed.value } else { speed.value };
        (v, speed.value * pursuit_curvature(local))
    }
}
