//! Monte Carlo pose correction using timestamped, associated range observations.

use glam::DVec2;
use ozton_diagnostics::log_row;
use ozton_math::{
    geometry::{BodyDelta, Pose},
    localization::{
        mcl::{Beam, BeamQuality, ParticleConfig, ParticleFilter},
        range::{Constraint, Rejection, fit_consensus},
    },
    path::clearance::WallContactConstraint,
    uom::si::f64::Angle,
};

use super::{MAX_RANGES, Observations, PoseEstimator, RangeRoles};

mod recovery;

/// Independent Monte Carlo pose correction with reusable transactional particle storage.
pub struct Mcl {
    filter: ParticleFilter,
    candidate: ParticleFilter,
    config: ParticleConfig,
    yaw_rate: f64,
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
        })
    }
}

impl PoseEstimator for Mcl {
    fn range_roles(&self) -> RangeRoles {
        RangeRoles::MCL
    }

    fn name(&self, shadow: bool) -> &'static str {
        if shadow {
            "Odometry / MCL shadow"
        } else {
            "MCL"
        }
    }
    fn position_covariance(&self) -> Option<glam::DMat2> {
        Some(self.filter.position_covariance())
    }
    fn reset(&mut self, pose: Pose) {
        self.filter.reset(pose);
        self.yaw_rate = 0.0;
    }
    fn reset_at_contact(&mut self, pose: Pose, contact: WallContactConstraint) {
        self.filter.reset_at_contact(pose, contact);
        self.yaw_rate = 0.0;
    }
    fn set_contact_constraint(&mut self, contact: Option<WallContactConstraint>) {
        self.filter.set_contact_constraint(contact);
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
        let mut constraints = [None; MAX_RANGES];
        let mut sensor_indices = [0; MAX_RANGES];
        let mut submitted_mask = 0u8;
        let mut residual_rejected = 0u8;
        let mut lengthening_rejected = false;
        let mut geometry_rejected = 0u8;
        let mut count = 0;
        for (index, (sensor, report)) in observations
            .sensors
            .iter()
            .zip(observations.reports)
            .enumerate()
        {
            let Some(report) = report else { continue };
            let Some(distance) = report.calibrated_distance else {
                continue;
            };
            // A particle determines its own association. The central pose's
            // missed ray or edge rejection does not invalidate a fresh sample.
            match report.constraint {
                Ok(_) | Err(Rejection::NearEdge | Rejection::Grazing) => {}
                Err(Rejection::NoSurface) if sensor.surface.is_none() => {}
                Err(Rejection::NoSurface | Rejection::WrongSurface) => {
                    geometry_rejected |= 1 << index;
                    continue;
                }
                Err(_) => continue,
            }
            if let Some(limit) = self.config.max_range_residual {
                // Local tracking with an association gate requires a valid
                // geometric constraint; grazing and edge returns are not
                // reliable innovations even when their range residual is small.
                let Ok(constraint) = report.constraint else {
                    geometry_rejected |= 1 << index;
                    continue;
                };
                // The report was already raycast at the same predicted pose
                // with the timestamp-aligned sensor mount. Reuse its residual.
                if constraint.residual.abs() > limit.value {
                    residual_rejected |= 1 << index;
                    lengthening_rejected |= constraint.residual > limit.value;
                    continue;
                }
            }
            let range_sigma = sensor
                .range_sigma(distance)
                .max(self.config.range_sigma.value);
            let gate = sensor.effective_gate();
            qualities[count] = BeamQuality {
                min_range: gate.min_range,
                sigma: ozton_math::qty!((range_sigma) m),
            };
            if let Ok(constraint) = report.constraint {
                constraints[count] =
                    Some((constraint, range_sigma * constraint.hit.incidence.abs()));
            }
            beams[count] = Beam {
                mount: sensor.mount,
                distance,
                max_range: gate.max_range,
                surface_mask: sensor.surface.map_or(sensor.surface_mask, |surface| {
                    sensor.surface_mask & 1u128.checked_shl(surface as u32).unwrap_or(0)
                }),
            };
            sensor_indices[count] = index;
            submitted_mask |= 1 << index;
            count += 1;
        }
        if lengthening_rejected
            && let Some(limit) = self.config.max_range_residual
            && let Some(maximum) = self.config.max_recovery_translation
            && let Some((pose, mask)) = recovery::recover(&observations, limit.value, maximum.value)
        {
            let change = pose.position - predicted.position;
            if self.filter.translate(change) {
                log_row!(
                    Trace,
                    "MCL",
                    "status=recovered,used={mask},dx_m={:.5},dy_m={:.5}",
                    change.x,
                    change.y
                );
                return Some(pose);
            }
        }
        if count == 0 {
            if residual_rejected != 0 || geometry_rejected != 0 {
                log_row!(
                    Trace,
                    "MCL_BEAMS",
                    "submitted=0,used=0,residual_rejected={residual_rejected},geometry_rejected={geometry_rejected}"
                );
            }
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
        // Use the measurements once. An accepted translation recovery above
        // returns immediately instead of using its beams to weight particles.
        self.candidate.clone_from(&self.filter);
        let estimate = self.candidate.observe_position_with_quality(
            &beams[..count],
            &qualities[..count],
            map,
            predicted.heading,
        );
        let used = self.candidate.last_used_beams();
        let mut used_mask = 0u8;
        let mut used_constraints = [(Constraint::default(), 0.0); MAX_RANGES];
        let mut constraint_count = 0;
        for index in 0..count {
            if used & (1 << index) != 0 {
                used_mask |= 1 << sensor_indices[index];
                if let Some(constraint) = constraints[index] {
                    used_constraints[constraint_count] = constraint;
                    constraint_count += 1;
                }
            }
        }
        log_row!(
            Trace,
            "MCL_BEAMS",
            "submitted={submitted_mask},used={used_mask},residual_rejected={residual_rejected},geometry_rejected={geometry_rejected}"
        );
        let rank =
            || fit_consensus(&used_constraints[..constraint_count]).map_or(0, |fit| fit.axes);
        let Some(estimate) = estimate else {
            log_row!(
                Trace,
                "MCL",
                "status=no_particle_estimate,beams={count},rank={}",
                rank()
            );
            return None;
        };
        let change = estimate.pose.position - predicted.position;
        let deviation = estimate.standard_deviation;
        let correction = change;
        let pose = estimate.pose;
        if !pose.is_finite() {
            log_row!(
                Trace,
                "MCL",
                "status=invalid_candidate,beams={count},rank={}",
                rank()
            );
            return None;
        }
        log_row!(
            Trace,
            "MCL",
            "status=estimated,beams={count},rank={},particle_x_m={:.5},particle_y_m={:.5},particle_heading_rad={:.5},sigma_x_m={:.5},sigma_y_m={:.5},gain=1.00000,dx_m={:.5},dy_m={:.5},candidate_x_m={:.5},candidate_y_m={:.5}",
            rank(),
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
