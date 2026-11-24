//! Timestamp-aware linear Kalman velocity estimation.
//!
//! Each axis estimates velocity and additive acceleration bias. For a measured
//! acceleration a, x = [v, b], F = [[1, -dt], [0, 1]], B = [dt, 0].
//! Without acceleration the model is a random walk in velocity. The measurement
//! is a direct rate when available, otherwise an odometry increment divided by dt.
//! See docs/src/localization.md for units and noise calibration.
use glam::DVec2;

#[derive(Debug, Clone, Copy, Default)]
pub struct MotionObservations {
    /// Independent chassis-right / chassis-forward acceleration at the origin,
    /// in m/s², with gravity and mount motion removed.
    pub acceleration: Option<DVec2>,
    /// Forward velocity from drive feedback, in m/s.
    pub forward_rate: Option<f64>,
    /// CCW angular rate, in rad/s (prefer the gyro).
    pub yaw_rate: Option<f64>,
}

/// Process noise and measurement standard deviations in SI units.
#[derive(Debug, Clone, Copy)]
pub struct VelocityConfig {
    /// Calibrated acceleration channels in chassis-right, forward order.
    pub acceleration_axes: [bool; 2],
    /// Velocity process variance per second, (m/s)²/s.
    pub linear_process: f64,
    /// Gyro-rate process variance per second, (rad/s)²/s.
    pub angular_process: f64,
    /// Acceleration bias random-walk variance per second, (m/s²)²/s.
    pub bias_process: f64,
    /// Standard deviation of direct wheel velocity, m/s.
    pub wheel_rate_sigma: f64,
    /// Standard deviation of gyro velocity, rad/s.
    pub gyro_rate_sigma: f64,
    /// Standard deviation of one wheel travel reading, m.
    pub travel_sigma: f64,
    /// Standard deviation of one heading reading, rad.
    pub heading_sigma: f64,
}

impl Default for VelocityConfig {
    fn default() -> Self {
        Self {
            acceleration_axes: [true; 2],
            linear_process: 0.50,
            angular_process: 2.0,
            bias_process: 0.01,
            wheel_rate_sigma: 0.08,
            gyro_rate_sigma: 0.05,
            travel_sigma: 0.0015,
            heading_sigma: 0.002,
        }
    }
}

impl VelocityConfig {
    pub fn valid(self) -> bool {
        [
            self.linear_process,
            self.angular_process,
            self.bias_process,
            self.wheel_rate_sigma,
            self.gyro_rate_sigma,
            self.travel_sigma,
            self.heading_sigma,
        ]
        .into_iter()
        .all(|v| v.is_finite() && v > 0.0)
    }
}

#[derive(Clone, Copy, Default)]
struct Axis {
    initialized: bool,
    rate: f64,
    bias: f64,
    p00: f64,
    p01: f64,
    p11: f64,
}

impl Axis {
    fn update(
        &mut self,
        dt: f64,
        acceleration: Option<f64>,
        measurement: f64,
        variance: f64,
        process: f64,
        bias_process: f64,
    ) -> f64 {
        if !self.initialized {
            *self = Self {
                initialized: true,
                rate: measurement,
                bias: 0.0,
                p00: variance,
                p01: 0.0,
                p11: 1.0,
            };
            return self.rate;
        }
        // Predict with F P F^T + Q. Bias is observable only with acceleration.
        let coupling = if let Some(a) = acceleration.filter(|a| a.is_finite()) {
            self.rate += (a - self.bias) * dt;
            -dt
        } else {
            0.0
        };
        self.p00 += 2.0 * coupling * self.p01 + coupling * coupling * self.p11 + process * dt;
        self.p01 += coupling * self.p11;
        self.p11 += bias_process * dt;

        // Scalar measurement update, H = [1, 0]. Use Joseph covariance
        // form to preserve positive semidefiniteness under rounding.
        let innovation = measurement - self.rate;
        let s = self.p00 + variance;
        let k0 = self.p00 / s;
        let k1 = self.p01 / s;
        self.rate += k0 * innovation;
        self.bias += k1 * innovation;
        let (p00, p01, p11) = (self.p00, self.p01, self.p11);
        let a = 1.0 - k0;
        self.p00 = a * a * p00 + k0 * k0 * variance;
        self.p01 = a * (p01 - k1 * p00) + k0 * k1 * variance;
        self.p11 = p11 - 2.0 * k1 * p01 + k1 * k1 * (p00 + variance);
        self.rate
    }
}

#[derive(Default)]
pub(super) struct VelocityEstimator {
    config: VelocityConfig,
    axes: [Axis; 3],
}

impl VelocityEstimator {
    pub(super) fn set_config(&mut self, config: VelocityConfig) -> bool {
        if !config.valid() {
            return false;
        }
        self.config = config;
        self.reset();
        true
    }

    pub(super) fn standard_deviation(&self) -> [f64; 3] {
        self.axes.map(|axis| axis.p00.max(0.0).sqrt())
    }

    pub(super) fn reset(&mut self) {
        self.axes = [Axis::default(); 3];
    }

    pub(super) fn update(
        &mut self,
        movement: ozton_math::geometry::BodyDelta,
        dt: f64,
        observations: MotionObservations,
    ) -> [f64; 3] {
        let c = self.config;
        let direct_forward = observations.forward_rate.filter(|v| v.is_finite());
        let direct_yaw = observations.yaw_rate.filter(|v| v.is_finite());
        // Variance of the difference of two position measurements, divided by dt².
        let travel_variance = 2.0 * (c.travel_sigma / dt).powi(2);
        let heading_variance = 2.0 * (c.heading_sigma / dt).powi(2);
        let yaw = direct_yaw.unwrap_or(movement.heading.value / dt);
        // In rotating chassis coordinates, dv_body/dt = a_body - omega x v.
        let acceleration = observations
            .acceleration
            .filter(|a| a.is_finite())
            .map(|a| a + DVec2::new(self.axes[1].rate * yaw, -self.axes[0].rate * yaw));
        [
            self.axes[0].update(
                dt,
                acceleration.filter(|_| c.acceleration_axes[0]).map(|a| a.x),
                movement.translation.x / dt,
                travel_variance,
                c.linear_process,
                c.bias_process,
            ),
            self.axes[1].update(
                dt,
                acceleration.filter(|_| c.acceleration_axes[1]).map(|a| a.y),
                direct_forward.unwrap_or(movement.translation.y / dt),
                if direct_forward.is_some() {
                    c.wheel_rate_sigma.powi(2)
                } else {
                    travel_variance
                },
                c.linear_process,
                c.bias_process,
            ),
            self.axes[2].update(
                dt,
                None,
                yaw,
                if direct_yaw.is_some() {
                    c.gyro_rate_sigma.powi(2)
                } else {
                    heading_variance
                },
                c.angular_process,
                c.bias_process,
            ),
        ]
    }
}

#[cfg(test)]
mod tests {
    use ozton_math::{geometry::BodyDelta, qty};

    use super::*;

    fn delta(forward: f64) -> BodyDelta {
        BodyDelta {
            translation: DVec2::new(0.0, forward),
            heading: qty!(0 rad),
        }
    }

    #[test]
    fn direct_velocity_does_not_inherit_encoder_packet_bursts() {
        let mut filter = VelocityEstimator::default();
        for travel in [0.00549, 0.002118, 0.008862, 0.00549] {
            let result = filter.update(
                delta(travel),
                0.01,
                MotionObservations {
                    forward_rate: Some(0.549),
                    ..MotionObservations::default()
                },
            );
            assert!((result[1] - 0.549).abs() < 1e-10);
        }
    }

    #[test]
    fn acceleration_prediction_tracks_a_ramp_and_estimates_bias() {
        let mut filter = VelocityEstimator::default();
        for i in 0..200 {
            let speed = i as f64 * 0.01;
            let result = filter.update(
                delta(speed * 0.01),
                0.01,
                MotionObservations {
                    acceleration: Some(DVec2::new(0.0, 1.2)),
                    forward_rate: Some(speed),
                    yaw_rate: Some(0.0),
                },
            );
            assert!((result[1] - speed).abs() < 0.03);
        }
        assert!((filter.axes[1].bias - 0.2).abs() < 0.15);
        filter.reset();
        assert_eq!(
            filter.update(delta(0.0), 0.01, MotionObservations::default()),
            [0.0; 3]
        );
    }
}
