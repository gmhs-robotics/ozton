//! Hardware-independent path tracking laws.
mod boomerang;
mod unicycle;
pub use boomerang::Boomerang;
pub use unicycle::{LtvLqr, PolarPose, Ramsete};

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
        let local = pose.to_local(target);
        let v = if reverse { -speed.value } else { speed.value };
        (v, v * pursuit_curvature(local))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::DVec2;

    #[test]
    fn reverse_pursuit_matches_rotated_travel_frame() {
        for heading in [-3.0, -1.0, 0.0, 1.0, 3.0] {
            let pose = Pose::new(DVec2::new(0.1, -0.2), crate::qty!((heading) rad));
            let target = DVec2::new(-0.4, 0.8);
            let speed = crate::qty!(1.2 m/s);
            let travel = Pose {
                heading: pose.heading + crate::qty!((core::f64::consts::PI) rad),
                ..pose
            };
            let expected = speed.value * pursuit_curvature(travel.to_local(target));
            let (v, yaw) = PurePursuit::new().twist(pose, target, speed, true);
            assert_eq!(v, -speed.value);
            assert!((yaw - expected).abs() < 1e-12);
        }
    }
}
