//! Three-state planar EKF with scalar measurement updates.

use glam::{DMat3, DVec2, DVec3};

use crate::{
    geometry::{BodyDelta, Pose, wrap_heading},
    path::clearance::WallContactConstraint,
};

/// Uncertainty is expressed in square meters and square radians.
#[derive(Debug, Clone, Copy)]
pub struct PoseEkf {
    pose: Pose,
    covariance: DMat3,
}

impl PoseEkf {
    pub fn new(pose: Pose, variance: DVec3) -> Option<Self> {
        if !pose.is_finite() || !variance.is_finite() || variance.min_element() < 0.0 {
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

    /// Condition the mean on a surveyed physical contact plane without
    /// pretending that uncertainty after motion is zero.
    pub fn constrain_contact(&mut self, contact: WallContactConstraint) {
        self.pose = contact.project(self.pose);
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
        let covariance =
            predict_covariance(self.covariance, travel) + DMat3::from_diagonal(process_variance);
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
        self.correct_impl(innovation, h, variance, gate_squared, false)
    }

    /// Position-only Schmidt update: heading remains owned by the odometry/IMU
    /// prediction. H may include heading uncertainty, but sparse position
    /// measurements cannot invent a new yaw. Joseph covariance update is retained.
    pub fn correct_position(
        &mut self,
        innovation: f64,
        h: DVec3,
        variance: f64,
        gate_squared: f64,
    ) -> bool {
        self.correct_impl(innovation, h, variance, gate_squared, true)
    }

    fn correct_impl(
        &mut self,
        innovation: f64,
        h: DVec3,
        variance: f64,
        gate_squared: f64,
        preserve_heading: bool,
    ) -> bool {
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
        let mut gain = pht / innovation_variance;
        if preserve_heading {
            gain.z = 0.0;
        }
        let correction = gain * innovation;
        let residual = DMat3::IDENTITY - outer(gain, h);
        let covariance =
            residual * self.covariance * residual.transpose() + outer(gain, gain) * variance;
        let pose = Pose {
            position: self.pose.position + DVec2::new(correction.x, correction.y),
            heading: wrap_heading(self.pose.heading + crate::qty!((correction.z) rad)),
        };
        if !pose.is_finite() || !covariance.is_finite() {
            return false;
        }
        self.pose = pose;
        self.covariance = (covariance + covariance.transpose()) * 0.5;
        true
    }
}

/// F = I + u e_z^T, where u = (-dy, dx, 0). Exploit its two off-diagonal
/// entries rather than evaluating two dense 3x3 matrix products.
fn predict_covariance(covariance: DMat3, travel: DVec2) -> DMat3 {
    let u = DVec3::new(-travel.y, travel.x, 0.0);
    let right_x = covariance.x_axis + covariance.z_axis * u.x;
    let right_y = covariance.y_axis + covariance.z_axis * u.y;
    let right_z = covariance.z_axis;
    let transform = |column: DVec3| column + u * column.z;
    DMat3::from_cols(transform(right_x), transform(right_y), transform(right_z))
}

fn outer(left: DVec3, right: DVec3) -> DMat3 {
    DMat3::from_cols(left * right.x, left * right.y, left * right.z)
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn sparse_prediction_matches_dense_jacobian() {
        let covariance = DMat3::from_cols(
            DVec3::new(0.08, 0.01, -0.002),
            DVec3::new(0.01, 0.05, 0.003),
            DVec3::new(-0.002, 0.003, 0.01),
        );
        for travel in [DVec2::ZERO, DVec2::new(0.2, -0.1), DVec2::new(-0.8, 0.4)] {
            let f = DMat3::from_cols(DVec3::X, DVec3::Y, DVec3::new(-travel.y, travel.x, 1.0));
            let reference = f * covariance * f.transpose();
            let actual = predict_covariance(covariance, travel);
            for (a, b) in actual.to_cols_array().into_iter().zip(reference.to_cols_array()) {
                assert!((a - b).abs() < 1e-14);
            }
        }
    }

    #[test]
    fn correction_reduces_error_and_rejects_outlier() {
        let mut filter = PoseEkf::new(
            Pose::new(
                crate::geometry::point(crate::qty!(0.12 m), crate::qty!(0.2 m)),
                crate::qty!(0.0 rad),
            ),
            DVec3::splat(0.01),
        )
        .unwrap();
        assert!(filter.correct(-0.12, DVec3::X, 0.0004, 9.0));
        assert!(filter.pose().position.x.abs() < 0.01);
        assert_eq!(filter.pose().position.y, 0.2);
        let covariance = filter.covariance();
        assert!(!filter.correct(2.0, DVec3::X, 0.0004, 9.0));
        assert_eq!(filter.covariance(), covariance);
        assert!(covariance.x_axis.x > 0.0);
    }
}
