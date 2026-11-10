//! Estimator selection and bounded acceptance of external corrections.

use glam::{DVec2, DVec3};
use ozton_math::{
    geometry::{BodyDelta, Pose, direction, wrap_heading},
    localization::{
        ekf::PoseEkf,
        mcl::{Beam, ParticleConfig, ParticleFilter},
        range::{Constraint, Surface},
    },
};

use super::{GpsFix, MAX_RANGES, RangeReport, RangeSensorConfig};

#[derive(Debug, Clone, Copy)]
pub struct EkfConfig {
    pub initial_variance: DVec3,
    pub process_variance_per_second: DVec3,
    pub gate_squared: f64,
    pub gps_max_error: f64,
}

impl Default for EkfConfig {
    fn default() -> Self {
        Self {
            initial_variance: DVec3::splat(0.01),
            process_variance_per_second: DVec3::new(0.002, 0.002, 0.001),
            gate_squared: 9.0,
            gps_max_error: 0.06,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Estimator {
    Odometry,
    Ekf(EkfConfig),
    Mcl(ParticleConfig),
}

pub(super) enum Filter {
    Odometry,
    Ekf(PoseEkf, EkfConfig),
    Mcl(ParticleFilter),
}

impl Filter {
    pub(super) fn new(pose: Pose, estimator: Estimator) -> Option<Self> {
        match estimator {
            Estimator::Odometry => Some(Self::Odometry),
            Estimator::Ekf(config)
                if config.process_variance_per_second.is_finite()
                    && config.process_variance_per_second.min_element() >= 0.0
                    && config.gate_squared.is_finite()
                    && config.gate_squared > 0.0
                    && config.gps_max_error.is_finite()
                    && config.gps_max_error > 0.0 =>
            {
                Some(Self::Ekf(
                    PoseEkf::new(pose, config.initial_variance)?,
                    config,
                ))
            }
            Estimator::Ekf(_) => None,
            Estimator::Mcl(config) => Some(Self::Mcl(ParticleFilter::new(pose, config)?)),
        }
    }

    pub(super) fn reset(&mut self, pose: Pose) {
        match self {
            Self::Odometry => {}
            Self::Ekf(filter, config) => {
                if let Some(reset) = PoseEkf::new(pose, config.initial_variance) {
                    *filter = reset;
                }
            }
            Self::Mcl(filter) => filter.reset(pose),
        }
    }

    pub(super) fn predict(&mut self, delta: BodyDelta, dt: f64) -> bool {
        match self {
            Self::Odometry => true,
            Self::Ekf(filter, config) => {
                filter.predict(delta, config.process_variance_per_second * dt)
            }
            Self::Mcl(filter) => {
                filter.predict(delta);
                true
            }
        }
    }

    pub(super) fn observe(
        &mut self,
        predicted: Pose,
        gps: Option<GpsFix>,
        sensors: &[RangeSensorConfig],
        reports: &[Option<RangeReport>; MAX_RANGES],
        map: &[Surface],
        maximum_correction: f64,
    ) -> Option<Pose> {
        let measurements = || {
            sensors.iter().zip(reports).filter_map(|(sensor, report)| {
                let report = report.as_ref()?;
                Some((sensor, report.constraint.ok()?, report.calibrated_distance?))
            })
        };
        match self {
            Self::Odometry => None,
            Self::Ekf(filter, config) => {
                let mut candidate = *filter;
                let mut changed = false;
                if let Some(gps) = gps.filter(|fix| {
                    fix.pose.is_finite()
                        && (0.001..=config.gps_max_error).contains(&fix.position_error)
                }) {
                    let variance = gps.position_error * gps.position_error;
                    changed |= candidate.correct(
                        gps.pose.position.x - candidate.pose().position.x,
                        DVec3::X,
                        variance,
                        config.gate_squared,
                    );
                    changed |= candidate.correct(
                        gps.pose.position.y - candidate.pose().position.y,
                        DVec3::Y,
                        variance,
                        config.gate_squared,
                    );
                }
                for (sensor, constraint, distance) in measurements() {
                    let normal = constraint.hit.normal;
                    let pose = candidate.pose();
                    let beam = direction(pose.heading + sensor.mount.heading);
                    let endpoint = pose.transform_point(sensor.mount.offset) + beam * distance;
                    let surface = map[constraint.hit.surface];
                    let h = DVec3::new(
                        normal.x,
                        normal.y,
                        normal.dot((endpoint - pose.position).perp()),
                    );
                    let error = normal.dot(surface.start - endpoint);
                    let sigma = sensor.sigma * constraint.hit.incidence.abs();
                    changed |= candidate.correct(error, h, sigma * sigma, config.gate_squared);
                }
                if changed && bounded(predicted, candidate.pose(), maximum_correction) {
                    *filter = candidate;
                    Some(candidate.pose())
                } else {
                    None
                }
            }
            Self::Mcl(filter) => {
                let empty = Beam {
                    mount: ozton_math::localization::range::Mount {
                        offset: DVec2::ZERO,
                        heading: 0.0,
                    },
                    distance: 0.0,
                    max_range: 0.0,
                };
                let mut beams = [empty; MAX_RANGES];
                let mut count = 0;
                for (sensor, _, distance) in measurements() {
                    beams[count] = Beam {
                        mount: sensor.mount,
                        distance,
                        max_range: sensor.gate.max_range,
                    };
                    count += 1;
                }
                let estimate = filter.observe(&beams[..count], map)?;
                let (first, second) =
                    independent(measurements().map(|(_, constraint, _)| constraint))?;
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
                Some(Pose {
                    position: predicted.position + correction * 0.4,
                    ..predicted
                })
            }
        }
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

fn bounded(from: Pose, to: Pose, maximum: f64) -> bool {
    to.is_finite()
        && from.position.distance(to.position) <= maximum
        && wrap_heading(to.heading - from.heading).abs() <= 0.12
}
