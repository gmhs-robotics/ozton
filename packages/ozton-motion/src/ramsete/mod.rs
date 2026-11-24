//! RAMSETE feedback for a time-parameterized differential path.

use ozton_math::{
    geometry::{Pose, wrap_heading},
    path::PathSample,
};

/// RAMSETE convergence gain and damping. Use a positive b and damping in (0, 1].
#[derive(Debug, Clone, Copy)]
pub struct Ramsete {
    pub b: f64,
    pub zeta: f64,
}

impl Default for Ramsete {
    fn default() -> Self {
        Self::new()
    }
}

impl Ramsete {
    pub const fn new() -> Self {
        Self { b: 2.0, zeta: 0.7 }
    }

    #[must_use]
    pub const fn with_b(mut self, b: f64) -> Self {
        self.b = b;
        self
    }

    #[must_use]
    pub const fn with_zeta(mut self, zeta: f64) -> Self {
        self.zeta = zeta;
        self
    }

    /// Desired forward velocity (m/s) and angular velocity (rad/s) for this pose and path sample.
    pub fn twist(self, pose: Pose, reference: PathSample, reverse: bool) -> (f64, f64) {
        let sign = if reverse { -1.0 } else { 1.0 };
        let facing = ozton_math::angle!(if reverse { core::f64::consts::PI } else { 0.0 }, rad);
        let error = pose.to_local(reference.pose.position);
        let heading_error = wrap_heading(reference.pose.heading + facing - pose.heading).value;
        let velocity = sign * reference.speed;
        let omega = reference.speed * reference.curvature;
        let gain = 2.0 * self.zeta * (omega * omega + self.b * velocity * velocity).sqrt();
        let sinc = if heading_error.abs() < 1e-5 {
            1.0
        } else {
            heading_error.sin() / heading_error
        };
        (
            velocity * heading_error.cos() + gain * error.y,
            omega + gain * heading_error + self.b * velocity * sinc * -error.x,
        )
    }
}
