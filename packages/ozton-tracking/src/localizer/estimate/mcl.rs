//! Monte Carlo pose correction using timestamped, associated range observations.

use glam::DVec2;
use ozton_math::{
    geometry::{BodyDelta, Pose},
    localization::{
        mcl::{Beam, ParticleConfig, ParticleFilter},
        range::{Constraint, Mount},
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
        let Observations {
            predicted,
            map,
            maximum_correction,
            ..
        } = observations;
        let empty = Beam {
            mount: Mount {
                offset: DVec2::ZERO,
                heading: 0.0,
            },
            distance: Length::new::<meter>(0.0),
            max_range: Length::new::<meter>(0.0),
            surface_mask: u32::MAX,
        };
        let mut beams = [empty; MAX_RANGES];
        let mut count = 0;
        let measurements = observations.measurements();
        for (sensor, _, distance) in measurements.clone() {
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
        self.candidate.clone_from(&self.filter);
        let estimate = self.candidate.observe_position(
            &beams[..count],
            map,
            Angle::new::<radian>(predicted.heading),
        )?;
        let (first, second) = independent(measurements.map(|(_, constraint, _)| constraint))?;
        let change = estimate.pose.position - predicted.position;
        let normal = first.hit.normal;
        let deviation = estimate.standard_deviation;
        let correction = if second.is_some() {
            if deviation.x.max(deviation.y) > maximum_correction * 0.5 {
                return None;
            }
            change
        } else {
            let spread = DVec2::new(deviation.x, deviation.y).dot(normal.abs());
            if spread > maximum_correction * 0.5 {
                return None;
            }
            normal * change.dot(normal)
        };
        if !correction.is_finite() || correction.length() > maximum_correction {
            return None;
        }
        core::mem::swap(&mut self.filter, &mut self.candidate);
        Some(Pose {
            position: predicted.position + correction * 0.4,
            ..predicted
        })
    }
}

fn independent(
    mut constraints: impl Iterator<Item = Constraint>,
) -> Option<(Constraint, Option<Constraint>)> {
    let first = constraints.next()?;
    let second =
        constraints.find(|candidate| first.hit.normal.perp_dot(candidate.hit.normal).abs() >= 0.5);
    Some((first, second))
}
