//! Monte Carlo pose correction using timestamped, associated range observations.

use glam::DVec2;
use ozton_diagnostics::log_row;
use ozton_math::{
    geometry::{BodyDelta, Pose},
    localization::{
        mcl::{Beam, ParticleConfig, ParticleFilter},
        range::{Constraint, fit_translation},
    },
    uom::si::{
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
            mount: Pose {
                position: DVec2::ZERO,
                heading: Angle::default(),
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
            log_row!(Trace, "MCL", "status=no_consistent_beams,beams=0");
            return None;
        }
        let Some((change, rank)) = fit_translation(&constraints[..count]) else {
            log_row!(Trace, "MCL", "status=translation_fit_failed,beams={count}");
            return None;
        };
        let normal = (rank == 1).then_some(constraints[0].0.hit.normal);
        self.candidate.clone_from(&self.filter);
        self.candidate
            .propose_position(predicted.position + change, normal, noise);
        let Some(estimate) =
            self.candidate
                .observe_position(&beams[..count], map, predicted.heading)
        else {
            log_row!(
                Trace,
                "MCL",
                "status=no_particle_estimate,beams={count},rank={rank}"
            );
            return None;
        };
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
            log_row!(
                Trace,
                "MCL",
                "status=invalid_spread_or_noise,beams={count},rank={rank},spread_m={spread:.5},noise_m={noise:.5}"
            );
            return None;
        }
        let signal = correction.length().hypot(noise);
        let gain = 0.4 * signal / (signal + spread);
        let pose = Pose {
            position: predicted.position + correction * gain,
            ..predicted
        };
        if !pose.is_finite() {
            log_row!(
                Trace,
                "MCL",
                "status=invalid_candidate,beams={count},rank={rank}"
            );
            return None;
        }
        log_row!(
            Trace,
            "MCL",
            "status=estimated,beams={count},rank={rank},particle_x_m={:.5},particle_y_m={:.5},particle_heading_rad={:.5},sigma_x_m={:.5},sigma_y_m={:.5},gain={gain:.5},dx_m={:.5},dy_m={:.5},candidate_x_m={:.5},candidate_y_m={:.5}",
            estimate.pose.position.x,
            estimate.pose.position.y,
            estimate.pose.heading.value,
            deviation.x,
            deviation.y,
            correction.x,
            correction.y,
            pose.position.x,
            pose.position.y
        );
        core::mem::swap(&mut self.filter, &mut self.candidate);
        Some(pose)
    }
}
