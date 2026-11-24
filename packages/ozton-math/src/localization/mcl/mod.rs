//! Bounded Monte Carlo localization with a robust range beam model.

mod random;
use alloc::vec::Vec;

use glam::{DVec2, DVec3};
use random::Random;
use uom::si::f64::{Angle, AngularVelocity, Length};

use super::range::{Surface, ray_distance};
use crate::geometry::{BodyDelta, Pose, bearing, direction, rotation, wrap_heading};

const MAX_PARTICLES: usize = 128;

#[derive(Debug, Clone, Copy)]
pub struct ParticleConfig {
    pub count: usize,
    pub seed: u64,
    /// Initial position and heading standard deviations.
    pub initial_spread: DVec3,
    /// Motion standard deviations per meter of travel or radian of rotation.
    pub motion_noise: DVec3,
    pub range_sigma: Length,
    pub outlier_weight: f64,
    /// Maximum difference between a range reading and the predicted wall distance.
    pub max_range_residual: Option<Length>,
    /// Defer live range corrections above this yaw speed while still predicting pose.
    pub max_correction_yaw_rate: Option<AngularVelocity>,
}

impl Default for ParticleConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl ParticleConfig {
    pub const DEFAULT: Self = Self {
        count: 64,
        seed: 1,
        initial_spread: DVec3::new(0.09, 0.09, 0.03),
        motion_noise: DVec3::new(0.04, 0.04, 0.03),
        range_sigma: crate::qty!(0.025 m),
        outlier_weight: 0.08,
        max_range_residual: None,
        max_correction_yaw_rate: None,
    };
}

#[derive(Debug, Clone, Copy)]
struct Particle {
    pose: Pose,
    weight: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct Beam {
    pub mount: Pose,
    pub distance: Length,
    pub max_range: Length,
    /// Eligible surfaces for this beam, including mounting-height eligibility.
    pub surface_mask: u128,
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

impl Clone for ParticleFilter {
    fn clone(&self) -> Self {
        let mut scratch = Vec::with_capacity(self.config.count);
        scratch.extend_from_slice(&self.scratch);
        Self {
            config: self.config,
            particles: self.particles.clone(),
            scratch,
            random: self.random.clone(),
        }
    }

    fn clone_from(&mut self, source: &Self) {
        self.config = source.config;
        self.particles.clone_from(&source.particles);
        self.scratch.clone_from(&source.scratch);
        self.random.clone_from(&source.random);
    }
}

impl ParticleFilter {
    pub fn new(pose: Pose, config: ParticleConfig) -> Option<Self> {
        if !(8..=MAX_PARTICLES).contains(&config.count)
            || !pose.is_finite()
            || !config.initial_spread.is_finite()
            || config.initial_spread.min_element() <= 0.0
            || !config.motion_noise.is_finite()
            || config.motion_noise.min_element() < 0.0
            || !config.range_sigma.value.is_finite()
            || config.range_sigma.value <= 0.0
            || !(0.001..=0.9).contains(&config.outlier_weight)
            || [
                config.max_range_residual.map(|limit| limit.value),
                config.max_correction_yaw_rate.map(|limit| limit.value),
            ]
            .into_iter()
            .flatten()
            .any(|limit| !limit.is_finite() || limit <= 0.0)
        {
            return None;
        }
        let mut filter = Self {
            config,
            particles: Vec::with_capacity(config.count),
            scratch: Vec::with_capacity(config.count),
            random: Random::seeded(config.seed),
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
                    pose.position + noise.truncate(),
                    wrap_heading(pose.heading + crate::qty!((noise.z) rad)),
                ),
                weight,
            });
        }
    }

    pub fn predict(&mut self, delta: BodyDelta) {
        if !delta.is_finite() {
            return;
        }
        let magnitude = delta.translation.length() + delta.heading.value.abs();
        if magnitude == 0.0 || !magnitude.is_finite() {
            return;
        }
        let spread = self.config.motion_noise * magnitude;
        for particle in &mut self.particles {
            let noise = self.random.normal3() * spread;
            particle.pose = particle.pose.integrate(BodyDelta {
                translation: delta.translation + DVec2::new(noise.x, noise.y),
                heading: delta.heading + crate::qty!((noise.z) rad),
            });
        }
    }

    /// Seed some particles from coherent external translation observations.
    /// A single wall normal leaves each particle's tangent coordinate intact.
    pub fn propose_position(&mut self, target: DVec2, normal: Option<DVec2>, noise: f64) {
        let estimate = self.estimate();
        let displacement = target - estimate.pose.position;
        let deviation = estimate.standard_deviation.truncate();
        let (displacement, spread) = if let Some(normal) = normal {
            (
                normal * displacement.dot(normal),
                deviation.dot(normal.abs()),
            )
        } else {
            (displacement, deviation.length())
        };
        if !noise.is_finite()
            || noise <= 0.0
            || !displacement.is_finite()
            || displacement.length() <= libm::hypot(spread, noise)
        {
            return;
        }
        let weight = 1.0 / self.particles.len() as f64;
        for particle in self.particles.iter_mut().step_by(4) {
            particle.pose.position += displacement;
            particle.weight = weight;
        }
        let total = self
            .particles
            .iter()
            .map(|particle| particle.weight)
            .sum::<f64>();
        for particle in &mut self.particles {
            particle.weight /= total;
        }
    }

    /// Conditions particle yaw on an independently tracked heading before a
    /// translation-only range update. Position-only output must not use particles
    /// whose yaw differs from the heading actually published to motion control.
    pub fn observe_position(
        &mut self,
        beams: &[Beam],
        map: &[Surface],
        heading: Angle,
    ) -> Option<Estimate> {
        if !heading.value.is_finite() {
            return None;
        }
        let heading = wrap_heading(heading);
        for particle in &mut self.particles {
            particle.pose.heading = heading;
        }
        self.observe_beams(beams, map, Some(rotation(heading)))
    }

    /// Updates each beam once, returning the estimate before any resampling.
    pub fn observe(&mut self, beams: &[Beam], map: &[Surface]) -> Option<Estimate> {
        self.observe_beams(beams, map, None)
    }

    fn observe_beams(
        &mut self,
        beams: &[Beam],
        map: &[Surface],
        fixed_rotation: Option<DVec2>,
    ) -> Option<Estimate> {
        if beams.is_empty()
            || map.is_empty()
            || beams.iter().any(|beam| {
                !beam.distance.value.is_finite()
                    || !beam.max_range.value.is_finite()
                    || beam.distance.value <= 0.0
                    || beam.distance > beam.max_range
                    || !beam.mount.is_finite()
            })
        {
            return None;
        }
        let sigma = self.config.range_sigma.value;
        let mixture = self.config.outlier_weight;
        for beam in beams {
            // Divide the mixture by the Gaussian peak. Normalization cancels
            // this constant and keeps tiny noise values from overflowing.
            let random_likelihood = ((sigma / beam.max_range.value)
                * (mixture * libm::sqrt(core::f64::consts::TAU) / (1.0 - mixture)))
                .clamp(1e-12, 1e12);
            let mut total = 0.0;
            let body_direction = direction(beam.mount.heading);
            for particle in &mut self.particles {
                let orientation = fixed_rotation.unwrap_or_else(|| rotation(particle.pose.heading));
                let origin = particle.pose.position + orientation.rotate(beam.mount.position);
                let direction = orientation.rotate(body_direction);
                let matched = ray_distance(origin, direction, map, beam.surface_mask).map_or(
                    0.0,
                    |distance| {
                        let residual = (beam.distance.value - distance) / sigma;
                        libm::exp(-0.5 * residual * residual)
                    },
                );
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
        let heading = bearing(headings);
        let mut variance = DVec3::ZERO;
        for particle in &self.particles {
            let error = particle.pose.position - position;
            let difference = DVec3::new(
                error.x,
                error.y,
                wrap_heading(particle.pose.heading - heading).value,
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

#[cfg(test)]
mod tests;
