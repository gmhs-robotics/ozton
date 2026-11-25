//! Bounded Monte Carlo localization with a robust range beam model.

mod random;
mod sensor;
use alloc::vec::Vec;

use glam::{DMat2, DVec2, DVec3};
use random::Random;
pub use sensor::BeamModel;
use uom::si::f64::{Angle, AngularVelocity, Length};

use super::range::{RayFace, Surface, prepare_faces, prepared_ray_distance};
use crate::{
    geometry::{BodyDelta, Pose, bearing, direction, rotation, wrap_heading},
    path::clearance::WallContactConstraint,
};

const MAX_PARTICLES: usize = 128;

#[derive(Debug, Clone, Copy)]
pub struct ParticleConfig {
    pub count: usize,
    pub seed: u64,
    /// Initial position and heading standard deviations.
    pub initial_spread: DVec3,
    /// Motion diffusion coefficients: variance grows with accumulated travel.
    pub motion_noise: DVec3,
    pub range_sigma: Length,
    pub outlier_weight: f64,
    pub beam_model: BeamModel,
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
        beam_model: BeamModel::DEFAULT,
        max_range_residual: None,
        max_correction_yaw_rate: None,
    };

    pub const fn new() -> Self {
        Self::DEFAULT
    }

    #[must_use]
    pub const fn with_particles(mut self, count: usize) -> Self {
        self.count = count;
        self
    }

    #[must_use]
    pub const fn with_beam_model(mut self, model: BeamModel) -> Self {
        self.beam_model = model;
        self
    }

    #[must_use]
    pub const fn with_max_range_residual(mut self, value: Length) -> Self {
        self.max_range_residual = Some(value);
        self
    }

    #[must_use]
    pub const fn with_max_correction_yaw_rate(mut self, value: AngularVelocity) -> Self {
        self.max_correction_yaw_rate = Some(value);
        self
    }
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

/// Per-beam physical lower bound and range-dependent noise. The upper
/// bound remains in Beam. Legacy callers can use the filter-wide noise.
#[derive(Debug, Clone, Copy)]
pub struct BeamQuality {
    pub min_range: Length,
    pub sigma: Length,
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
    ray_faces: Vec<RayFace>,
    contact: Option<WallContactConstraint>,
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
            ray_faces: Vec::with_capacity(self.ray_faces.capacity()),
            contact: self.contact,
        }
    }

    fn clone_from(&mut self, source: &Self) {
        self.config = source.config;
        self.particles.clone_from(&source.particles);
        self.scratch.clone_from(&source.scratch);
        self.random.clone_from(&source.random);
        self.contact = source.contact;
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
            || !config.beam_model.is_valid()
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
            ray_faces: Vec::new(),
            contact: None,
        };
        filter.reset(pose);
        Some(filter)
    }

    pub fn reset(&mut self, pose: Pose) {
        self.contact = None;
        self.reset_with_spread(pose, self.config.initial_spread);
    }

    /// A verified flush contact fixes wall-normal position and heading at
    /// initialization. Tangential placement retains its surveyed uncertainty.
    /// Normal prediction diffusion resumes with the next measured motion.
    pub fn reset_at_contact(&mut self, pose: Pose, contact: WallContactConstraint) {
        self.contact = Some(contact);
        let mut spread = self.config.initial_spread;
        let axis = contact.wall().axis();
        spread[axis] = 0.0;
        spread[1 - axis] = contact.tangential_sigma().value;
        spread.z = 0.0;
        self.reset_with_spread(pose, spread);
        self.set_contact_constraint(Some(contact));
    }

    pub fn set_contact_constraint(&mut self, contact: Option<WallContactConstraint>) {
        self.contact = contact;
        if let Some(contact) = contact {
            for particle in &mut self.particles {
                particle.pose = contact.project(particle.pose);
            }
        }
    }

    fn reset_with_spread(&mut self, pose: Pose, spread: DVec3) {
        if !pose.is_finite() {
            return;
        }
        self.particles.clear();
        let weight = 1.0 / self.config.count as f64;
        for _ in 0..self.config.count {
            let noise = self.random.normal3() * spread;
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
        let spread = self.config.motion_noise * libm::sqrt(magnitude);
        for particle in &mut self.particles {
            let noise = self.random.normal3() * spread;
            particle.pose = particle.pose.integrate(BodyDelta {
                translation: delta.translation + DVec2::new(noise.x, noise.y),
                heading: delta.heading + crate::qty!((noise.z) rad),
            });
            if let Some(contact) = self.contact {
                particle.pose = contact.project(particle.pose);
            }
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
            if let Some(contact) = self.contact {
                particle.pose = contact.project(particle.pose);
            }
        }
        self.observe_beams(beams, map, Some(rotation(heading)), None)
    }

    /// Updates each beam once, returning the estimate before any resampling.
    pub fn observe(&mut self, beams: &[Beam], map: &[Surface]) -> Option<Estimate> {
        self.observe_beams(beams, map, None, None)
    }

    /// Position update using independently calibrated quality for each beam.
    pub fn observe_position_with_quality(
        &mut self,
        beams: &[Beam],
        quality: &[BeamQuality],
        map: &[Surface],
        heading: Angle,
    ) -> Option<Estimate> {
        if !heading.value.is_finite() || beams.len() != quality.len() {
            return None;
        }
        let heading = wrap_heading(heading);
        for particle in &mut self.particles {
            particle.pose.heading = heading;
            if let Some(contact) = self.contact {
                particle.pose = contact.project(particle.pose);
            }
        }
        self.observe_beams(beams, map, Some(rotation(heading)), Some(quality))
    }

    fn observe_beams(
        &mut self,
        beams: &[Beam],
        map: &[Surface],
        fixed_rotation: Option<DVec2>,
        quality: Option<&[BeamQuality]>,
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
        prepare_faces(map, &mut self.ray_faces);
        let mut orientations = [DVec2::ZERO; MAX_PARTICLES];
        if fixed_rotation.is_none() {
            for (orientation, particle) in orientations.iter_mut().zip(&self.particles) {
                *orientation = rotation(particle.pose.heading);
            }
        }
        let mut likelihoods = [f64::NAN; MAX_PARTICLES];
        let mut used = false;
        for (index, beam) in beams.iter().enumerate() {
            let (minimum, sigma) =
                quality.map_or((0.0, self.config.range_sigma.value), |quality| {
                    let q = quality[index];
                    (
                        q.min_range.value,
                        q.sigma.value.max(self.config.range_sigma.value),
                    )
                });
            if !minimum.is_finite()
                || minimum < 0.0
                || minimum >= beam.max_range.value
                || !sigma.is_finite()
                || sigma <= 0.0
                || beam.distance.value < minimum
                || quality.is_some_and(|q| {
                    !q[index].sigma.value.is_finite() || q[index].sigma.value <= 0.0
                })
            {
                continue;
            }
            let model = self.config.beam_model.prepare(
                beam.distance.value,
                minimum,
                beam.max_range.value,
                sigma,
                self.config.outlier_weight,
            );
            let body_direction = direction(beam.mount.heading);
            let fixed_ray =
                fixed_rotation.map(|r| (r.rotate(beam.mount.position), r.rotate(body_direction)));
            let mut modeled_weight = 0.0;
            let mut evidence = 0.0;
            let mut hit_evidence = 0.0;
            for (index, particle) in self.particles.iter().enumerate() {
                let (offset, direction) = fixed_ray.unwrap_or_else(|| {
                    let r = orientations[index];
                    (r.rotate(beam.mount.position), r.rotate(body_direction))
                });
                let origin = particle.pose.position + offset;
                let distance = prepared_ray_distance(
                    origin,
                    direction,
                    &self.ray_faces,
                    beam.surface_mask,
                    beam.max_range.value,
                );
                likelihoods[index] = if let Some(distance) = distance.filter(|d| *d >= minimum) {
                    let (hit, likelihood) = model.likelihood(distance);
                    modeled_weight += particle.weight;
                    hit_evidence += particle.weight * hit;
                    evidence += particle.weight * likelihood;
                    likelihood
                } else {
                    f64::NAN
                };
            }
            if modeled_weight <= 0.0
                || evidence <= 0.0
                || hit_evidence < evidence * self.config.beam_model.minimum_hit_probability
            {
                continue;
            }
            // Marginalize unknown geometry: a missing prediction receives
            // the conditional mean likelihood of modeled particles. This
            // keeps its prior mass unchanged, rather than rewarding or
            // punishing a particle for seeing through an opening.
            let mean = evidence / modeled_weight;
            for (particle, &likelihood) in self.particles.iter_mut().zip(&likelihoods) {
                if likelihood.is_finite() {
                    particle.weight *= likelihood / mean;
                }
            }
            used = true;
        }
        if !used {
            return None;
        }
        let mut estimate = if fixed_rotation.is_some() {
            let (position, deviation) = self.position_moments();
            Estimate {
                pose: Pose::new(position, self.particles[0].pose.heading),
                standard_deviation: deviation.extend(0.0),
            }
        } else {
            self.estimate(&orientations)
        };
        if let Some(contact) = self.contact {
            estimate.pose = contact.project(estimate.pose);
        }
        let squared_weight = self
            .particles
            .iter()
            .map(|p| p.weight * p.weight)
            .sum::<f64>();
        if squared_weight * self.config.count as f64 > 2.0 {
            self.resample();
        }
        Some(estimate)
    }

    /// Weighted posterior XY covariance in square metres.
    pub fn position_covariance(&self) -> DMat2 {
        let mean = self.position_mean();
        self.particles.iter().fold(DMat2::ZERO, |sum, p| {
            let d = p.pose.position - mean;
            sum + DMat2::from_cols(d * d.x, d * d.y) * p.weight
        })
    }

    fn position_mean(&self) -> DVec2 {
        self.particles.iter().fold(DVec2::ZERO, |sum, particle| {
            sum + particle.pose.position * particle.weight
        })
    }

    fn position_moments(&self) -> (DVec2, DVec2) {
        let position = self.position_mean();
        let variance = self.particles.iter().fold(DVec2::ZERO, |sum, particle| {
            let error = particle.pose.position - position;
            sum + error * error * particle.weight
        });
        (
            position,
            DVec2::new(libm::sqrt(variance.x), libm::sqrt(variance.y)),
        )
    }

    fn estimate(&self, orientations: &[DVec2; MAX_PARTICLES]) -> Estimate {
        let mut position = DVec2::ZERO;
        let mut headings = DVec2::ZERO;
        for (particle, orientation) in self.particles.iter().zip(orientations) {
            position += particle.pose.position * particle.weight;
            headings += DVec2::new(-orientation.y, orientation.x) * particle.weight;
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
