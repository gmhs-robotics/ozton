//! Three-state planar EKF with scalar measurement updates.

use glam::{DMat3, DVec2, DVec3};

use crate::geometry::{BodyDelta, Pose, wrap_heading};

/// Uncertainty is expressed in square meters and square radians.
#[derive(Debug, Clone, Copy)]
pub struct PoseEkf {
    pose: Pose,
    covariance: DMat3,
}

impl PoseEkf {
    pub fn new(pose: Pose, variance: DVec3) -> Option<Self> {
        if !pose.is_finite() || !variance.is_finite() || variance.min_element() <= 0.0 {
            return None;
        }
        Some(Self {
            pose,
            covariance: DMat3::from_diagonal(variance),
        })
    }

    pub fn pose(&self) -> Pose {
        self.pose
    }

    pub fn covariance(&self) -> DMat3 {
        self.covariance
    }

    pub fn predict(&mut self, delta: BodyDelta, process_variance: DVec3) -> bool {
        if !delta.is_finite()
            || !process_variance.is_finite()
            || process_variance.min_element() < 0.0
        {
            return false;
        }
        let pose = self.pose.integrate(delta);
        let travel = pose.position - self.pose.position;
        let jacobian = DMat3::from_cols(DVec3::X, DVec3::Y, DVec3::new(-travel.y, travel.x, 1.0));
        let covariance = jacobian * self.covariance * jacobian.transpose()
            + DMat3::from_diagonal(process_variance);
        if !pose.is_finite() || !covariance.is_finite() {
            return false;
        }
        self.pose = pose;
        self.covariance = covariance;
        true
    }

    /// Applies a scalar innovation with observation Jacobian H and a squared sigma gate.
    /// A rejected observation leaves both the mean and covariance intact.
    pub fn correct(&mut self, innovation: f64, h: DVec3, variance: f64, gate_squared: f64) -> bool {
        let pht = self.covariance * h;
        let innovation_variance = h.dot(pht) + variance;
        if !innovation.is_finite()
            || !variance.is_finite()
            || variance <= 0.0
            || !gate_squared.is_finite()
            || gate_squared <= 0.0
            || !innovation_variance.is_finite()
            || innovation_variance <= 0.0
            || innovation * innovation / innovation_variance > gate_squared
        {
            return false;
        }
        let gain = pht / innovation_variance;
        let correction = gain * innovation;
        let residual = DMat3::IDENTITY - outer(gain, h);
        let covariance =
            residual * self.covariance * residual.transpose() + outer(gain, gain) * variance;
        let pose = Pose {
            position: self.pose.position + DVec2::new(correction.x, correction.y),
            heading: wrap_heading(self.pose.heading + correction.z),
        };
        if !pose.is_finite() || !covariance.is_finite() {
            return false;
        }
        self.pose = pose;
        self.covariance = (covariance + covariance.transpose()) * 0.5;
        true
    }
}

fn outer(left: DVec3, right: DVec3) -> DMat3 {
    DMat3::from_cols(left * right.x, left * right.y, left * right.z)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn correction_reduces_error_and_rejects_outlier() {
        let mut filter = PoseEkf::new(Pose::new(0.12, 0.2, 0.0), DVec3::splat(0.01)).unwrap();
        assert!(filter.correct(-0.12, DVec3::X, 0.0004, 9.0));
        assert!(filter.pose().position.x.abs() < 0.01);
        assert_eq!(filter.pose().position.y, 0.2);
        let covariance = filter.covariance();
        assert!(!filter.correct(2.0, DVec3::X, 0.0004, 9.0));
        assert_eq!(filter.covariance(), covariance);
        assert!(covariance.x_axis.x > 0.0);
    }
}
