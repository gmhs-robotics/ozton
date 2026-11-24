//! Monte Carlo pose correction using timestamped, associated range observations.

use glam::DVec2;
use ozton_diagnostics::log_row;
use ozton_math::{
    geometry::{BodyDelta, Pose},
    localization::{
        mcl::{Beam, BeamQuality, ParticleConfig, ParticleFilter},
        range::{Constraint, Rejection, cast_masked, fit_consensus},
    },
    uom::si::f64::Angle,
};

use super::{MAX_RANGES, Observations, PoseEstimator};

/// Independent Monte Carlo pose correction with reusable transactional particle storage.
pub struct Mcl {
    filter: ParticleFilter,
    candidate: ParticleFilter,
    config: ParticleConfig,
    yaw_rate: f64,
    correction_gain: f64,
}

impl Mcl {
    pub fn new(pose: Pose, config: ParticleConfig) -> Option<Self> {
        let filter = ParticleFilter::new(pose, config)?;
        let candidate = filter.clone();
        Some(Self {
            filter,
            candidate,
            config,
            yaw_rate: 0.0,
            correction_gain: 1.0,
        })
    }
}

impl Mcl {
    pub fn set_correction_gain(&mut self, gain: f64) -> bool {
        if !(0.0..=1.0).contains(&gain) {
            return false;
        }
        self.correction_gain = gain;
        true
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
        self.yaw_rate = 0.0;
    }
    fn predict(&mut self, delta: BodyDelta, seconds: f64) -> bool {
        self.yaw_rate = delta.heading.value.abs() / seconds;
        let _span =
            ozton_diagnostics::profile::Span::new(ozton_diagnostics::profile::Stage::MclPredict);
        self.filter.predict(delta);
        true
    }
    fn observe(&mut self, observations: Observations<'_>) -> Option<Pose> {
        if self
            .config
            .max_correction_yaw_rate
            .is_some_and(|limit| self.yaw_rate > limit.value)
        {
            log_row!(
                Trace,
                "MCL",
                "status=turning,yaw_radps={:.5}",
                self.yaw_rate
            );
            return None;
        }
        let _span =
            ozton_diagnostics::profile::Span::new(ozton_diagnostics::profile::Stage::MclObserve);
        let Observations { predicted, map, .. } = observations;
        let empty = Beam {
            mount: Pose {
                position: DVec2::ZERO,
                heading: Angle::default(),
            },
            distance: ozton_math::qty!(0.0 m),
            max_range: ozton_math::qty!(0.0 m),
            surface_mask: u128::MAX,
        };
        let mut beams = [empty; MAX_RANGES];
        let mut qualities = [BeamQuality {
            min_range: ozton_math::qty!(0 m),
            sigma: self.config.range_sigma,
        }; MAX_RANGES];
        let mut constraints = [(Constraint::default(), 0.0); MAX_RANGES];
        let mut count = 0;
        let mut constraint_count = 0;
        for (sensor, report) in observations.sensors.iter().zip(observations.reports) {
            let Some(report) = report else { continue };
            let Some(distance) = report.calibrated_distance else {
                continue;
            };
            let hit = match report.constraint {
                Ok(constraint) => constraint.hit,
                // Edge and incidence gates protect the linearized wall fit, not
                // the nonlinear particle likelihood.
                Err(Rejection::NearEdge | Rejection::Grazing) => {
                    let Some(hit) = cast_masked(predicted, sensor.mount, map, sensor.surface_mask)
                    else {
                        continue;
                    };
                    hit
                }
                Err(_) => continue,
            };
            if self
                .config
                .max_range_residual
                .is_some_and(|limit| (distance.value - hit.distance).abs() > limit.value)
            {
                continue;
            }
            let range = ozton_math::qty!((distance.value.max(hit.distance)) m);
            let range_sigma = sensor.range_sigma(range).max(self.config.range_sigma.value);
            let gate = sensor.effective_gate();
            qualities[count] = BeamQuality {
                min_range: gate.min_range,
                sigma: ozton_math::qty!((range_sigma) m),
            };
            if let Ok(constraint) = report.constraint {
                constraints[constraint_count] = (constraint, range_sigma * hit.incidence.abs());
                constraint_count += 1;
            }
            beams[count] = Beam {
                mount: sensor.mount,
                distance,
                max_range: gate.max_range,
                surface_mask: sensor.surface_mask,
            };
            count += 1;
        }
        if count == 0 {
            let mut no_surface = 0;
            let mut no_new_packet = 0;
            let mut residual = 0;
            let mut quality = 0;
            for report in observations.reports.iter().flatten() {
                match report.constraint {
                    Ok(_) => residual += 1,
                    Err(Rejection::NoSurface) => no_surface += 1,
                    Err(Rejection::Missing | Rejection::Duplicate) => no_new_packet += 1,
                    Err(_) => quality += 1,
                }
            }
            let status = if no_surface == 0 && residual == 0 && quality == 0 {
                "no_fresh_beams"
            } else {
                "no_consistent_beams"
            };
            log_row!(
                Trace,
                "MCL",
                "status={status},beams=0,no_surface={no_surface},no_new_packet={no_new_packet},residual_rejected={residual},quality_rejected={quality}"
            );
            return None;
        }
        // Local Bayesian tracking must not create particles from the same
        // measurement used to weight them. Explicit DSR/set_pose is the
        // separate recovery path for a pose outside the motion prior.
        let rank = fit_consensus(&constraints[..constraint_count]).map_or(0, |fit| fit.axes);
        self.candidate.clone_from(&self.filter);
        let Some(estimate) = self.candidate.observe_position_with_quality(
            &beams[..count],
            &qualities[..count],
            map,
            predicted.heading,
        ) else {
            log_row!(
                Trace,
                "MCL",
                "status=no_particle_estimate,beams={count},rank={rank}"
            );
            return None;
        };
        let change = estimate.pose.position - predicted.position;
        let deviation = estimate.standard_deviation;
        let correction = change;
        let gain = self.correction_gain;
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
