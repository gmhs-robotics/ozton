//! Gated range corrections with an extended Kalman filter.

use glam::DVec3;
use ozton_math::{
    geometry::{BodyDelta, Pose, direction},
    localization::ekf::PoseEkf,
};

use super::{Observations, PoseEstimator};

#[derive(Debug, Clone, Copy)]
pub struct EkfConfig {
    pub initial_variance: DVec3,
    pub process_variance_per_second: DVec3,
    pub gate_squared: f64,
}

impl Default for EkfConfig {
    fn default() -> Self {
        Self {
            initial_variance: DVec3::splat(0.01),
            process_variance_per_second: DVec3::new(0.002, 0.002, 0.001),
            gate_squared: 9.0,
        }
    }
}

/// Independent extended Kalman pose correction, using ranges.
pub struct Ekf {
    filter: PoseEkf,
    config: EkfConfig,
}

impl Ekf {
    pub fn new(pose: Pose, config: EkfConfig) -> Option<Self> {
        if !config.process_variance_per_second.is_finite()
            || config.process_variance_per_second.min_element() < 0.0
            || !config.gate_squared.is_finite()
            || config.gate_squared <= 0.0
        {
            return None;
        }
        Some(Self {
            filter: PoseEkf::new(pose, config.initial_variance)?,
            config,
        })
    }
}

impl PoseEstimator for Ekf {
    fn name(&self, shadow: bool) -> &'static str {
        if shadow {
            "Odometry / EKF shadow"
        } else {
            "EKF"
        }
    }
    fn reset(&mut self, pose: Pose) {
        if let Some(filter) = PoseEkf::new(pose, self.config.initial_variance) {
            self.filter = filter;
        }
    }
    fn predict(&mut self, delta: BodyDelta, seconds: f64) -> bool {
        self.filter
            .predict(delta, self.config.process_variance_per_second * seconds)
    }
    fn observe(&mut self, observations: Observations<'_>) -> Option<Pose> {
        let Observations { map, .. } = observations;
        let config = self.config;
        let mut candidate = self.filter;
        let mut changed = false;
        for (sensor, constraint, distance) in observations.measurements(None) {
            let normal = constraint.hit.normal;
            let pose = candidate.pose();
            let beam = direction(pose.heading + sensor.mount.heading);
            let endpoint = pose.transform_point(sensor.mount.position) + beam * distance;
            let surface = map[constraint.hit.surface];
            // The published orientation is independently tracked. Condition
            // this wall update on that orientation instead of treating sparse
            // translation beams as an observation of all three pose states.
            let h = DVec3::new(normal.x, normal.y, 0.0);
            let error = normal.dot(surface.start - endpoint);
            let sigma = sensor.sigma.value * constraint.hit.incidence.abs();
            changed |= candidate.correct_position(error, h, sigma * sigma, config.gate_squared);
        }
        if changed && candidate.pose().is_finite() {
            self.filter = candidate;
            Some(candidate.pose())
        } else {
            None
        }
    }
}
