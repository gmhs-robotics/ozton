//! Bounded Monte Carlo localization with a robust range beam model.

use alloc::vec::Vec;

use glam::{DVec2, DVec3};

use super::range::{Mount, Surface, cast};
use crate::geometry::{BodyDelta, Pose, direction, wrap_heading};

const MAX_PARTICLES: usize = 128;

#[derive(Debug, Clone, Copy)]
pub struct ParticleConfig {
    pub count: usize,
    pub seed: u64,
    /// Initial position and heading standard deviations.
    pub initial_spread: DVec3,
    /// Motion standard deviations per meter of travel or radian of rotation.
    pub motion_noise: DVec3,
    pub range_sigma: f64,
    pub outlier_weight: f64,
}

impl Default for ParticleConfig {
    fn default() -> Self {
        Self {
            count: 64,
            seed: 1,
            initial_spread: DVec3::new(0.09, 0.09, 0.03),
            motion_noise: DVec3::new(0.04, 0.04, 0.03),
            range_sigma: 0.025,
            outlier_weight: 0.08,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct Particle {
    pose: Pose,
    weight: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct Beam {
    pub mount: Mount,
    pub distance: f64,
    pub max_range: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct Estimate {
    pub pose: Pose,
    pub standard_deviation: DVec3,
}

pub struct ParticleFilter {
    config: ParticleConfig,
    particles: Vec<Particle>,
    scratch: Vec<Particle>,
    random: Random,
}

impl ParticleFilter {
    pub fn new(pose: Pose, config: ParticleConfig) -> Option<Self> {
        if !(8..=MAX_PARTICLES).contains(&config.count)
            || !pose.is_finite()
            || !config.initial_spread.is_finite()
            || config.initial_spread.min_element() <= 0.0
            || !config.motion_noise.is_finite()
            || config.motion_noise.min_element() < 0.0
            || !config.range_sigma.is_finite()
            || config.range_sigma <= 0.0
            || !(0.001..=0.9).contains(&config.outlier_weight)
        {
            return None;
        }
        let mut filter = Self {
            config,
            particles: Vec::with_capacity(config.count),
            scratch: Vec::with_capacity(config.count),
            random: Random(config.seed.max(1)),
        };
        filter.reset(pose);
        Some(filter)
    }

    pub fn reset(&mut self, pose: Pose) {
        if !pose.is_finite() {
            return;
        }
        self.particles.clear();
        let weight = 1.0 / self.config.count as f64;
        for _ in 0..self.config.count {
            let noise = self.random.normal3() * self.config.initial_spread;
            self.particles.push(Particle {
                pose: Pose::new(
                    pose.position.x + noise.x,
                    pose.position.y + noise.y,
                    wrap_heading(pose.heading + noise.z),
                ),
                weight,
            });
        }
    }

    pub fn predict(&mut self, delta: BodyDelta) {
        if !delta.is_finite() {
            return;
        }
        let magnitude = delta.translation.length() + delta.heading.abs();
        if magnitude == 0.0 || !magnitude.is_finite() {
            return;
        }
        let spread = self.config.motion_noise * magnitude;
        for particle in &mut self.particles {
            let noise = self.random.normal3() * spread;
            particle.pose = particle.pose.integrate(BodyDelta {
                translation: delta.translation + DVec2::new(noise.x, noise.y),
                heading: delta.heading + noise.z,
            });
        }
    }

    /// Updates each beam once, returning the estimate before any resampling.
    pub fn observe(&mut self, beams: &[Beam], map: &[Surface]) -> Option<Estimate> {
        if beams.is_empty()
            || map.is_empty()
            || beams.iter().any(|beam| {
                !beam.distance.is_finite()
                    || !beam.max_range.is_finite()
                    || beam.distance <= 0.0
                    || beam.distance > beam.max_range
                    || !beam.mount.offset.is_finite()
                    || !beam.mount.heading.is_finite()
            })
        {
            return None;
        }
        let sigma = self.config.range_sigma;
        let mixture = self.config.outlier_weight;
        for beam in beams {
            // Divide the mixture by the Gaussian peak. Normalization cancels
            // this constant and keeps tiny noise values from overflowing.
            let random_likelihood = ((sigma / beam.max_range)
                * (mixture * libm::sqrt(core::f64::consts::TAU) / (1.0 - mixture)))
                .clamp(1e-12, 1e12);
            let mut total = 0.0;
            for particle in &mut self.particles {
                let matched = cast(particle.pose, beam.mount, map).map_or(0.0, |hit| {
                    let residual = (beam.distance - hit.distance) / sigma;
                    libm::exp(-0.5 * residual * residual)
                });
                particle.weight *= matched + random_likelihood;
                total += particle.weight;
            }
            for particle in &mut self.particles {
                particle.weight /= total;
            }
        }
        let estimate = self.estimate();
        let effective_count = 1.0
            / self
                .particles
                .iter()
                .map(|p| p.weight * p.weight)
                .sum::<f64>();
        if effective_count < self.config.count as f64 * 0.5 {
            self.resample();
        }
        Some(estimate)
    }

    fn estimate(&self) -> Estimate {
        let mut position = DVec2::ZERO;
        let mut headings = DVec2::ZERO;
        for particle in &self.particles {
            position += particle.pose.position * particle.weight;
            headings += direction(particle.pose.heading) * particle.weight;
        }
        let heading = libm::atan2(headings.y, headings.x);
        let mut variance = DVec3::ZERO;
        for particle in &self.particles {
            let error = particle.pose.position - position;
            let difference = DVec3::new(
                error.x,
                error.y,
                wrap_heading(particle.pose.heading - heading),
            );
            variance += difference * difference * particle.weight;
        }
        Estimate {
            pose: Pose { position, heading },
            standard_deviation: DVec3::new(
                libm::sqrt(variance.x),
                libm::sqrt(variance.y),
                libm::sqrt(variance.z),
            ),
        }
    }

    fn resample(&mut self) {
        let step = 1.0 / self.config.count as f64;
        let start = self.random.uniform() * step;
        let mut index = 0;
        let mut cumulative = self.particles[0].weight;
        self.scratch.clear();
        for slot in 0..self.config.count {
            let target = start + slot as f64 * step;
            while target > cumulative && index + 1 < self.config.count {
                index += 1;
                cumulative += self.particles[index].weight;
            }
            self.scratch.push(Particle {
                pose: self.particles[index].pose,
                weight: step,
            });
        }
        core::mem::swap(&mut self.particles, &mut self.scratch);
    }
}

struct Random(u64);

impl Random {
    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        ((self.0 >> 12) as f64 + 0.5) / ((1_u64 << 52) as f64)
    }

    fn normal_pair(&mut self) -> DVec2 {
        let radius = libm::sqrt(-2.0 * libm::log(self.uniform()));
        direction(core::f64::consts::TAU * self.uniform()) * radius
    }

    fn normal3(&mut self) -> DVec3 {
        let xy = self.normal_pair();
        DVec3::new(xy.x, xy.y, self.normal_pair().x)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orthogonal_beams_reduce_seeded_initial_error() {
        let map = [
            Surface {
                start: DVec2::new(1.0, -1.0),
                end: DVec2::ONE,
            },
            Surface {
                start: DVec2::ONE,
                end: DVec2::new(-1.0, 1.0),
            },
        ];
        let beams = [0.0, core::f64::consts::FRAC_PI_2].map(|heading| Beam {
            mount: Mount {
                offset: DVec2::ZERO,
                heading,
            },
            distance: 1.0,
            max_range: 2.0,
        });
        let start = Pose::new(0.09, 0.08, 0.0);
        let mut filter = ParticleFilter::new(start, ParticleConfig::default()).unwrap();
        let estimate = filter.observe(&beams, &map).unwrap();
        assert!(estimate.pose.position.length() < start.position.length() * 0.6);
    }
}
