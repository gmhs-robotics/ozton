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

/// Finite-horizon, discrete linear time-varying unicycle LQR.
/// The reference is linearized at each timed sample, including curvature.
/// Diagonal Q and R are positive cost weights, not feedback gains.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LtvLqr {
    state: [f64; 3],
    input: [f64; 2],
}

/// Precomputed feedback gains for a time-parameterized trajectory.
pub struct LqrSchedule {
    gains: alloc::vec::Vec<[glam::DVec3; 2]>,
    period: f64,
}

impl Default for LtvLqr {
    fn default() -> Self {
        Self::new([4.0, 16.0, 4.0], [1.0, 1.0]).unwrap()
    }
}

impl LtvLqr {
    pub fn new(state: [f64; 3], input: [f64; 2]) -> Option<Self> {
        if !state
            .into_iter()
            .chain(input)
            .all(|x| x.is_finite() && x > 0.0)
        {
            return None;
        }
        Some(Self { state, input })
    }

    pub const fn weights(self) -> ([f64; 3], [f64; 2]) {
        (self.state, self.input)
    }

    /// Backward Riccati recursion with zero-order-hold discretization.
    /// Reference returns signed forward velocity and counterclockwise yaw rate.
    pub fn prepare(
        self,
        duration: f64,
        period: f64,
        reference: impl Fn(f64) -> [f64; 2],
    ) -> Option<LqrSchedule> {
        use glam::{DMat3, DVec3};
        if !duration.is_finite() || duration <= 0.0 || !period.is_finite() || period <= 0.0 {
            return None;
        }
        let count = libm::ceil(duration / period) as usize;
        if count == 0 {
            return None;
        }
        let mut gains = alloc::vec::Vec::new();
        gains.try_reserve_exact(count).ok()?;
        gains.resize(count, [DVec3::ZERO; 2]);
        let q = DMat3::from_diagonal(DVec3::from_array(self.state));
        let mut p = q;
        for i in (0..count).rev() {
            let t = i as f64 * period;
            let dt = period;
            let [v, w] = reference(t);
            let a = DMat3::from_cols(
                DVec3::new(0.0, -w, 0.0),
                DVec3::new(w, 0.0, 0.0),
                DVec3::new(0.0, v, 0.0),
            );
            // exp(A dt) and the integral of exp(A t), using the same
            // convergent series. A is frozen over one control interval.
            let mut term = DMat3::IDENTITY;
            let mut ad = term;
            let mut integral = term * dt;
            for n in 1..=12 {
                term = (term * a) * (dt / n as f64);
                ad += term;
                integral += term * (dt / (n + 1) as f64);
            }
            let b0 = integral.x_axis;
            let b1 = integral.z_axis;
            let pb0 = p * b0;
            let pb1 = p * b1;
            let s00 = self.input[0] * dt + b0.dot(pb0);
            let s01 = b0.dot(pb1);
            let s11 = self.input[1] * dt + b1.dot(pb1);
            let det = s00 * s11 - s01 * s01;
            if !det.is_finite() || det <= 0.0 {
                return None;
            }
            let g0 = ad.transpose() * pb0;
            let g1 = ad.transpose() * pb1;
            let k0 = (g0 * s11 - g1 * s01) / det;
            let k1 = (g1 * s00 - g0 * s01) / det;
            gains[i] = [k0, k1];
            let outer = |a: DVec3, b: DVec3| DMat3::from_cols(a * b.x, a * b.y, a * b.z);
            p = q * dt + ad.transpose() * p * ad - outer(g0, k0) - outer(g1, k1);
            p = (p + p.transpose()) * 0.5;
        }
        Some(LqrSchedule { gains, period })
    }
}

impl LqrSchedule {
    /// Feedforward plus optimal linear feedback in body coordinates.
    pub fn calculate(&self, time: f64, error: [f64; 3], reference: [f64; 2]) -> [f64; 2] {
        let i = ((time / self.period) as usize).min(self.gains.len() - 1);
        let e = glam::DVec3::from_array(error);
        let [kv, kw] = self.gains[i];
        [reference[0] + kv.dot(e), reference[1] + kw.dot(e)]
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
        Self {
            k_rho: 2.0,
            k_alpha: 4.0,
            k_beta: -1.0,
        }
    }
}

impl PolarPose {
    pub const fn new(k_rho: f64, k_alpha: f64, k_beta: f64) -> Option<Self> {
        if k_rho.is_finite()
            && k_alpha.is_finite()
            && k_beta.is_finite()
            && k_rho > 0.0
            && k_alpha > k_rho
            && k_beta < 0.0
        {
            Some(Self {
                k_rho,
                k_alpha,
                k_beta,
            })
        } else {
            None
        }
    }

    pub const fn gains(self) -> (f64, f64, f64) {
        (self.k_rho, self.k_alpha, self.k_beta)
    }

    /// Textbook polar-coordinate law: v = k_rho*rho,
    /// omega = k_alpha*alpha + k_beta*beta. All errors use SI units.
    pub fn twist(self, rho: f64, alpha: f64, beta: f64, reverse: bool) -> [f64; 2] {
        let speed = self.k_rho * rho;
        [
            if reverse { -speed } else { speed },
            self.k_alpha * alpha + self.k_beta * beta,
        ]
    }

    /// Constrained pose adapter. The stopping bound and heading projection
    /// are application constraints, not part of the polar feedback law.
    pub fn calculate(self, pose: Pose, target: Pose, reverse: bool, max_speed: f64) -> [f64; 2] {
        let delta = target.position - pose.position;
        let facing = crate::qty!((if reverse { 180.0 } else { 0.0 }) deg);
        let direction = bearing(delta);
        let alpha = wrap_heading(direction - pose.heading - facing).value;
        let beta = wrap_heading(target.heading + facing - direction).value;
        let [linear, angular] = self.twist(delta.length(), alpha, beta, reverse);
        let speed = linear.abs().min(max_speed) * libm::cos(alpha).max(0.0);
        [if reverse { -speed } else { speed }, angular]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_reference_and_reverse_sign() {
        let ramsete = Ramsete::default();
        for reference in [[0.8, 1.2], [-0.8, 1.2], [0.0, 0.0]] {
            assert_eq!(ramsete.calculate([0.0; 3], reference), reference);
            let lqr = LtvLqr::default().prepare(1.0, 0.01, |_| reference).unwrap();
            assert_eq!(lqr.calculate(0.0, [0.0; 3], reference), reference);
        }
        for v in [0.6_f64, -0.6] {
            let error = [0.0, 0.05, 0.0];
            assert_eq!(ramsete.calculate(error, [v, 0.0])[1].signum(), v.signum());
            let lqr = LtvLqr::default().prepare(2.0, 0.01, |_| [v, 0.0]).unwrap();
            assert_eq!(lqr.calculate(0.0, error, [v, 0.0])[1].signum(), v.signum());
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
