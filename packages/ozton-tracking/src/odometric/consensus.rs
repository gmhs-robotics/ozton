//! Continuous side odometry from all drive motors, with reconnect baselines.

use ozton_math::uom::si::{f64::Velocity, velocity::meter_per_second};

use super::WheelFeedback;

pub const MAX_SIDE_MOTORS: usize = 16;

/// Integrates median wheel travel, never averages unrelated accumulated zeros.
/// One missing/rejoining encoder cannot jump the side position. Majority disagreement
/// returns unavailable feedback so the localizer can latch loss of pose trust.
#[derive(Debug)]
pub struct EncoderConsensus {
    previous: [Option<f64>; MAX_SIDE_MOTORS],
    position: f64,
}
impl Default for EncoderConsensus {
    fn default() -> Self {
        Self {
            previous: [None; MAX_SIDE_MOTORS],
            position: 0.0,
        }
    }
}
impl EncoderConsensus {
    pub fn sample(&mut self, samples: &[Option<WheelFeedback>]) -> Option<WheelFeedback> {
        if samples.len() > MAX_SIDE_MOTORS {
            return None;
        }
        let mut deltas = [0.0; MAX_SIDE_MOTORS];
        let mut speeds = [0.0; MAX_SIDE_MOTORS];
        let had_baseline = self.previous.iter().any(Option::is_some);
        let mut count = 0;
        let mut velocity_count = 0;
        for (index, sample) in samples.iter().enumerate() {
            let valid =
                sample.filter(|s| s.position.value.is_finite() && s.velocity.value.is_finite());
            if let Some(sample) = valid {
                speeds[velocity_count] = sample.velocity.value;
                velocity_count += 1;
                if let Some(last) = self.previous[index] {
                    let delta = sample.position.value - last;
                    if delta.is_finite() {
                        deltas[count] = delta;
                        count += 1;
                    }
                }
            }
            self.previous[index] = valid.map(|s| s.position.value);
        }
        if velocity_count == 0 || (had_baseline && count == 0) {
            return None;
        }
        speeds[..velocity_count].sort_unstable_by(f64::total_cmp);
        let velocity = median(&speeds[..velocity_count]);
        if count > 0 {
            deltas[..count].sort_unstable_by(f64::total_cmp);
            let delta = median(&deltas[..count]);
            let tolerance = 0.005 + 0.3 * delta.abs();
            let agrees = deltas[..count]
                .iter()
                .filter(|v| (**v - delta).abs() <= tolerance)
                .count();
            if agrees * 2 <= count {
                return None;
            }
            self.position += delta;
        }
        Some(WheelFeedback {
            position: ozton_math::qty!((self.position) m),
            velocity: Velocity::new::<meter_per_second>(velocity),
        })
    }
}
fn median(values: &[f64]) -> f64 {
    let middle = values.len() / 2;
    if values.len() % 2 == 0 {
        (values[middle - 1] + values[middle]) * 0.5
    } else {
        values[middle]
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn sample(position: f64) -> Option<WheelFeedback> {
        Some(WheelFeedback {
            position: ozton_math::qty!((position) m),
            velocity: Velocity::new::<meter_per_second>(0.1),
        })
    }
    #[test]
    fn separate_zeros_loss_rejoin_and_outlier_preserve_continuous_travel() {
        let mut side = EncoderConsensus::default();
        assert_eq!(
            side.sample(&[sample(10.0), sample(-3.0), sample(5.0)])
                .unwrap()
                .position
                .value,
            0.0
        );
        assert!(
            (side
                .sample(&[sample(10.01), sample(-2.99), sample(5.9)])
                .unwrap()
                .position
                .value
                - 0.01)
                .abs()
                < 1e-9
        );
        assert!(
            (side
                .sample(&[None, sample(-2.98), sample(5.91)])
                .unwrap()
                .position
                .value
                - 0.02)
                .abs()
                < 1e-9
        );
        assert!(
            (side
                .sample(&[sample(0.0), sample(-2.97), sample(5.92)])
                .unwrap()
                .position
                .value
                - 0.03)
                .abs()
                < 1e-9
        );
        assert!(side.sample(&[None, None, None]).is_none());
        assert!(
            (side
                .sample(&[sample(2.0), sample(5.0), sample(-1.0)])
                .unwrap()
                .position
                .value
                - 0.03)
                .abs()
                < 1e-9
        );
    }
    #[test]
    fn two_disagreeing_encoders_are_unavailable() {
        let mut side = EncoderConsensus::default();
        side.sample(&[sample(0.0), sample(0.0)]);
        assert!(side.sample(&[sample(0.1), sample(0.0)]).is_none());
    }
}
