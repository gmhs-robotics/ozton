//! Allocation-free unicycle control laws, independent of hardware and scheduling.

use crate::{
    geometry::{Pose, bearing, wrap_heading},
    numeric::sqrt,
};

/// Nonlinear RAMSETE trajectory feedback (SI units).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ramsete {
    b: f64,
    zeta: f64,
}

impl Default for Ramsete {
    fn default() -> Self {
        Self { b: 2.0, zeta: 0.7 }
    }
}

impl Ramsete {
    pub const fn new(b: f64, zeta: f64) -> Option<Self> {
        if b.is_finite() && b > 0.0 && zeta > 0.0 && zeta <= 1.0 {
            Some(Self { b, zeta })
        } else {
            None
        }
    }

    pub const fn parameters(self) -> (f64, f64) {
        (self.b, self.zeta)
    }

    /// Body-frame error: forward, left, counterclockwise heading.
    /// Reference and output: signed forward m/s, counterclockwise rad/s.
    pub fn calculate(self, error: [f64; 3], reference: [f64; 2]) -> [f64; 2] {
        let [x, y, theta] = error;
        let [v, w] = reference;
        let gain = 2.0 * self.zeta * sqrt(w * w + self.b * v * v);
        let sinc = if theta.abs() < 1e-6 {
            1.0 - theta * theta / 6.0
        } else {
            libm::sin(theta) / theta
        };
        [
            v * libm::cos(theta) + gain * x,
            w + gain * theta + self.b * v * sinc * y,
        ]
    }
}

/// Continuous-time gain-scheduled LQR for the straight-reference unicycle
/// linearization. Not the full discrete curved-reference LTV controller.
/// The instantaneous Riccati solution has a closed form; constant gains
/// are computed once and no matrix solver runs in the control loop.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LtvLqr {
    state: [f64; 3],
    input: [f64; 2],
    kx: f64,
    ky: f64,
    heading_cost: f64,
}

impl Default for LtvLqr {
    fn default() -> Self {
        Self::new([4.0, 16.0, 4.0], [1.0, 1.0]).unwrap()
    }
}

impl LtvLqr {
    /// Diagonal positive Q and R cost weights in SI units.
    pub fn new(state: [f64; 3], input: [f64; 2]) -> Option<Self> {
        if !state.into_iter().chain(input).all(|x| x.is_finite() && x > 0.0) {
            return None;
        }
        let [qx, qy, qt] = state;
        let [rv, rw] = input;
        let kx = sqrt(qx / rv);
        let ky = sqrt(qy / rw);
        let heading_cost = qt / rw;
        if ![kx, ky, heading_cost].into_iter().all(f64::is_finite) {
            return None;
        }
        Some(Self { state, input, kx, ky, heading_cost })
    }

    pub const fn weights(self) -> ([f64; 3], [f64; 2]) {
        (self.state, self.input)
    }

    /// Body-frame error and reference twist use the same convention as RAMSETE.
    pub fn calculate(self, error: [f64; 3], reference: [f64; 2]) -> [f64; 2] {
        let [x, y, theta] = error;
        let [v, w] = reference;
        let kt = sqrt(self.heading_cost + 2.0 * v.abs() * self.ky);
        [
            v + self.kx * x,
            w + v.signum() * self.ky * y + kt * theta,
        ]
    }
}

/// Polar-coordinate pose stabilization. Wheel limits and arrival handling
/// are deliberately left to the actuator layer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PolarPose {
    k_rho: f64,
    k_alpha: f64,
    k_beta: f64,
}

impl Default for PolarPose {
    fn default() -> Self {
        Self { k_rho: 2.0, k_alpha: 4.0, k_beta: -1.0 }
    }
}

impl PolarPose {
    pub const fn new(k_rho: f64, k_alpha: f64, k_beta: f64) -> Option<Self> {
        if k_rho.is_finite() && k_alpha.is_finite() && k_beta.is_finite()
            && k_rho > 0.0 && k_alpha > k_rho && k_beta < 0.0
        {
            Some(Self { k_rho, k_alpha, k_beta })
        } else {
            None
        }
    }

    pub const fn gains(self) -> (f64, f64, f64) {
        (self.k_rho, self.k_alpha, self.k_beta)
    }

    /// Forward/yaw request with a caller-supplied stopping-speed bound.
    pub fn calculate(self, pose: Pose, target: Pose, reverse: bool, max_speed: f64) -> [f64; 2] {
        let delta = target.position - pose.position;
        let facing = crate::qty!((if reverse { 180.0 } else { 0.0 }) deg);
        let direction = bearing(delta);
        let alpha = wrap_heading(direction - pose.heading - facing).value;
        let beta = wrap_heading(target.heading + facing - direction).value;
        let speed = (self.k_rho * delta.length()).min(max_speed) * libm::cos(alpha).max(0.0);
        [
            if reverse { -speed } else { speed },
            self.k_alpha * alpha + self.k_beta * beta,
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_reference_and_reverse_sign() {
        let ramsete = Ramsete::default();
        let lqr = LtvLqr::default();
        for reference in [[0.8, 1.2], [-0.8, 1.2], [0.0, 0.0]] {
            assert_eq!(ramsete.calculate([0.0; 3], reference), reference);
            assert_eq!(lqr.calculate([0.0; 3], reference), reference);
        }
        for v in [0.6_f64, -0.6] {
            let error = [0.0, 0.05, 0.0];
            assert_eq!(ramsete.calculate(error, [v, 0.0])[1].signum(), v.signum());
            assert_eq!(lqr.calculate(error, [v, 0.0])[1].signum(), v.signum());
        }
    }

    #[test]
    fn invalid_parameters_are_rejected() {
        assert!(Ramsete::new(-1.0, 0.7).is_none());
        assert!(LtvLqr::new([1.0, 0.0, 1.0], [1.0, 1.0]).is_none());
        assert!(PolarPose::new(2.0, 1.0, -1.0).is_none());
    }

    #[test]
    fn polar_forward_and_reverse_are_consistent() {
        let controller = PolarPose::default();
        let target = Pose::new(glam::DVec2::Y, crate::qty!(0 deg));
        let start = Pose::default();
        assert_eq!(controller.calculate(start, target, false, 0.5), [0.5, 0.0]);
        let start = Pose::new(glam::DVec2::Y * 2.0, crate::qty!(0 deg));
        let output = controller.calculate(start, target, true, 0.5);
        assert!((output[0] + 0.5).abs() < 1e-12);
        assert!(output[1].abs() < 1e-12);
    }
}
