//! Monte Carlo pose correction using timestamped, associated range observations.

use glam::DVec2;
use ozton_math::{
    geometry::{BodyDelta, Pose},
    localization::{
        mcl::{Beam, ParticleConfig, ParticleFilter},
        range::{Constraint, Mount, fit_translation},
    },
    uom::si::{
        angle::radian,
        f64::{Angle, Length},
        length::meter,
    },
};

use super::{MAX_RANGES, Observations, PoseEstimator};

/// Independent Monte Carlo pose correction with reusable transactional particle storage.
pub struct Mcl {
    filter: ParticleFilter,
    candidate: ParticleFilter,
}

impl Mcl {
    pub fn new(pose: Pose, config: ParticleConfig) -> Option<Self> {
        let filter = ParticleFilter::new(pose, config)?;
        let candidate = filter.clone();
        Some(Self { filter, candidate })
    }
}

impl PoseEstimator for Mcl {
    fn name(&self, shadow: bool) -> &'static str {
        if shadow {
            "Odometry / MCL shadow"
        } else {
            "MCL"
        }
    }
    fn reset(&mut self, pose: Pose) {
        self.filter.reset(pose);
    }
    fn predict(&mut self, delta: BodyDelta, _: f64) -> bool {
        self.filter.predict(delta);
        true
    }
    fn observe(&mut self, observations: Observations<'_>) -> Option<Pose> {
        let Observations { predicted, map, .. } = observations;
        let empty = Beam {
            mount: Mount {
                offset: DVec2::ZERO,
                heading: 0.0,
            },
            distance: Length::new::<meter>(0.0),
            max_range: Length::new::<meter>(0.0),
            surface_mask: u128::MAX,
        };
        let mut beams = [empty; MAX_RANGES];
        let mut constraints = [(Constraint::default(), 0.0); MAX_RANGES];
        let mut noise = f64::INFINITY;
        let mut count = 0;
        let measurements = observations.measurements();
        for (sensor, constraint, distance) in measurements {
            let sigma = sensor.sigma.value * constraint.hit.incidence.abs();
            constraints[count] = (constraint, sigma);
            noise = noise.min(sigma);
            beams[count] = Beam {
                mount: sensor.mount,
                distance: Length::new::<meter>(distance),
                max_range: sensor.gate.max_range,
                surface_mask: sensor.surface_mask,
            };
            count += 1;
        }
        if count == 0 {
            return None;
        }
        let (change, rank) = fit_translation(&constraints[..count])?;
        let normal = (rank == 1).then_some(constraints[0].0.hit.normal);
        self.candidate.clone_from(&self.filter);
        self.candidate
            .propose_position(predicted.position + change, normal, noise);
        let estimate = self.candidate.observe_position(
            &beams[..count],
            map,
            Angle::new::<radian>(predicted.heading),
        )?;
        let change = estimate.pose.position - predicted.position;
        let deviation = estimate.standard_deviation;
        // Weight the correction by particle concentration relative to the beams'
        // measurement noise. A lone normal leaves translation along the wall intact.
        let (correction, spread) = if let Some(normal) = normal {
            (
                normal * change.dot(normal),
                DVec2::new(deviation.x, deviation.y).dot(normal.abs()),
            )
        } else {
            (change, deviation.x.max(deviation.y))
        };
        if !correction.is_finite() || !spread.is_finite() || !noise.is_finite() || noise <= 0.0 {
            return None;
        }
        let signal = correction.length().hypot(noise);
        let gain = 0.4 * signal / (signal + spread);
        let pose = Pose {
            position: predicted.position + correction * gain,
            ..predicted
        };
        if !pose.is_finite() {
            return None;
        }
        core::mem::swap(&mut self.filter, &mut self.candidate);
        Some(pose)
    }
}
