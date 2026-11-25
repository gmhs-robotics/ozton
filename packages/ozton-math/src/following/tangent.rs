//! Spatial tangent feedback used by the profiled path tracker.
//! This is Ozton's empirical geometric feedback law, not textbook Stanley.

use crate::{
    geometry::{Pose, wrap_heading},
    uom::si::f64::Angle,
};

#[derive(Debug, Default, Clone, Copy)]
pub struct Tangent;

impl Tangent {
    pub const fn new() -> Self {
        Self
    }

    /// Heading error with a cross-track correction over a spatial horizon.
    /// Reference heading is in the path's forward travel direction.
    pub fn heading_error(self, pose: Pose, reference: Pose, facing: Angle, lookahead: f64) -> f64 {
        let cross = reference.to_local(pose.position).x;
        let desired = reference.heading + facing + crate::qty!((libm::atan2(cross, lookahead)) rad);
        wrap_heading(desired - pose.heading).value
    }
}
