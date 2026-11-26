//! Local translation recovery using redundant, independently checked range geometry.

use ozton_math::{
    geometry::Pose,
    localization::{
        observation::{RangeObservation, RangeSensorConfig},
        range::{Constraint, fit_consensus},
    },
};

use super::{MAX_RANGES, Observations};
use crate::localizer::ranges::constrain_sample;

#[derive(Clone, Copy)]
struct Measurement {
    index: usize,
    sensor: RangeSensorConfig,
    sample: RangeObservation,
    constraint: Constraint,
    sigma: f64,
}

/// Search the full set, then all single exclusions. Each eligible solution must
/// explain every long return and may discard one short (potentially blocked) beam.
pub(super) fn recover(
    observations: &Observations<'_>,
    gate: f64,
    maximum: f64,
) -> Option<(Pose, u8)> {
    let measurements = std::array::from_fn::<_, MAX_RANGES, _>(|index| {
        let sensor = *observations.sensors.get(index)?;
        let report = observations.reports[index]?;
        let constraint = report.constraint.ok()?;
        Some(Measurement {
            index,
            sensor,
            sample: report.observation?,
            constraint,
            sigma: sensor.constraint_sigma(constraint, report.calibrated_distance?.value),
        })
    });
    let available = measurements.iter().flatten();
    let count = available.clone().count();
    if count < 3 {
        return None;
    }
    let solve = |excluded| candidate(observations, available.clone(), excluded, gate, maximum);
    if let Some(fit) = solve(None) {
        return Some(fit);
    }
    if count == 3 {
        return None;
    }
    let mut candidates = available
        .clone()
        .filter_map(|measurement| solve(Some(measurement.index)));
    let fit = candidates.next()?;
    candidates.next().is_none().then_some(fit)
}

fn candidate<'a>(
    observations: &Observations<'_>,
    measurements: impl Iterator<Item = &'a Measurement> + Clone,
    excluded: Option<usize>,
    gate: f64,
    maximum: f64,
) -> Option<(Pose, u8)> {
    let selected = measurements
        .clone()
        .filter(|measurement| Some(measurement.index) != excluded);
    let mut constraints = [(Constraint::default(), 0.0); MAX_RANGES];
    let mut count = 0;
    for m in selected.clone() {
        constraints[count] = (m.constraint, m.sigma);
        count += 1;
    }
    let fit = fit_consensus(&constraints[..count])?;
    if fit.axes != 2
        || fit.mask.count_ones() as usize != count
        || !(gate * gate..=maximum * maximum).contains(&fit.change.length_squared())
    {
        return None;
    }

    // Occlusion shortens a return. Treat a large shortening as informative
    // only if an opposing face independently shows a corresponding lengthening.
    // A single clear transverse beam can still constrain the other axis.
    let toward = |c: Constraint| c.hit.incidence.signum() * c.hit.normal;
    for m in selected.clone() {
        let c = m.constraint;
        if c.residual < -gate
            && !selected.clone().any(|other| {
                other.constraint.residual > gate && toward(c).dot(toward(other.constraint)) < -0.9
            })
        {
            return None;
        }
    }
    if !selected.clone().any(|m| m.constraint.residual > gate) {
        return None;
    }

    let mut pose = observations.predicted;
    pose.position += fit.change;
    if !pose.is_finite() {
        return None;
    }
    let mut mask = 0;
    for m in measurements {
        // The samples were already checked for age, freshness and quality.
        // Their mounts carry the odometry alignment to this observation tick.
        let checked = constrain_sample(pose, observations.map, m.sensor, m.sample, 0.0).ok()?;
        if Some(m.index) == excluded {
            if checked.residual >= -gate {
                return None;
            }
        } else {
            if checked.hit.surface != m.constraint.hit.surface || checked.residual.abs() > gate {
                return None;
            }
            mask |= 1 << m.index;
        }
    }
    Some((pose, mask))
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use ozton_math::{geometry::DVec2, localization::range::cast, qty};

    use super::*;
    use crate::{field::Field, localizer::RangeReport};

    fn run(truth: DVec2, obstructed: &[usize]) -> Option<(Pose, u8)> {
        let map: Vec<_> = Field::rectangle(qty!(3 m), qty!(3 m), qty!(30 cm))
            .unwrap()
            .surface_map()
            .unwrap()
            .faces()
            .collect();
        let sensors = [0.0, 90.0, 180.0, -90.0]
            .map(|heading| RangeSensorConfig::new(Pose::new(DVec2::ZERO, qty!((heading) deg))));
        let predicted = Pose::default();
        let truth = Pose::new(truth, predicted.heading);
        let mut reports = [None; MAX_RANGES];
        for (index, &sensor) in sensors.iter().enumerate() {
            let distance = if obstructed.contains(&index) {
                0.7
            } else {
                cast(truth, sensor.mount, &map).unwrap().distance
            };
            let sample = RangeObservation {
                timestamp: Duration::ZERO,
                distance: qty!((distance) m),
                confidence: 1.0,
                occluded: false,
            };
            reports[index] = Some(RangeReport {
                observation: Some(sample),
                calibrated_distance: Some(sample.distance),
                constraint: constrain_sample(predicted, &map, sensor, sample, 0.0),
            });
        }
        recover(
            &Observations {
                predicted,
                sensors: &sensors,
                reports: &reports,
                map: &map,
            },
            0.05,
            0.30,
        )
    }

    #[test]
    fn recovers_drift_even_when_one_beam_is_blocked() {
        let truth = DVec2::new(0.15, 0.0);
        for (obstructed, expected) in [(&[][..], 0b1111), (&[2][..], 0b1011)] {
            let (pose, mask) = run(truth, obstructed).unwrap();
            assert!(pose.position.distance(truth) < 1e-12);
            assert_eq!(pose.heading.value, 0.0);
            assert_eq!(mask, expected);
        }
    }

    #[test]
    fn rejects_occluders_and_drift_beyond_the_recovery_window() {
        assert!(run(DVec2::ZERO, &[0, 3]).is_none());
        assert!(run(DVec2::new(0.40, 0.0), &[]).is_none());
    }

    #[test]
    fn override_estimator_recovers_with_a_blocked_range() {
        use ozton_math::{
            localization::{mcl::ParticleConfig, range::cast_masked},
            position_xyzh,
        };

        use super::super::{Mcl, PoseEstimator};
        use crate::field::OVERRIDE;

        let map: Vec<_> = OVERRIDE
            .surface_map(Field::PORTABLE)
            .unwrap()
            .faces()
            .collect();
        let sensors = [
            position_xyzh!(0 cm, 13 cm, 92 mm, 0 deg),
            position_xyzh!(-13 cm, 0 cm, 206 mm, 90 deg),
            position_xyzh!(0 cm, -13 cm, 92 mm, 180 deg),
            position_xyzh!(13 cm, 0 cm, 206 mm, -90 deg),
        ]
        .map(|mount| RangeSensorConfig::at_mount(mount).with_visible_surfaces(&map));
        let predicted = Pose::new(DVec2::new(0.25, 0.22), qty!(0 deg));
        let truth = Pose::new(DVec2::new(0.10, 0.22), predicted.heading);
        let mut reports = [None; MAX_RANGES];
        for (index, &sensor) in sensors.iter().enumerate() {
            let distance = if index == 0 {
                0.7
            } else {
                cast_masked(truth, sensor.mount, &map, sensor.surface_mask)
                    .unwrap()
                    .distance
            };
            let sample = RangeObservation {
                timestamp: Duration::ZERO,
                distance: qty!((distance) m),
                confidence: 1.0,
                occluded: false,
            };
            reports[index] = Some(RangeReport {
                observation: Some(sample),
                calibrated_distance: Some(sample.distance),
                constraint: constrain_sample(predicted, &map, sensor, sample, 0.0),
            });
        }
        let config = ParticleConfig::new()
            .with_max_range_residual(qty!(5 cm))
            .with_max_recovery_translation(qty!(30 cm));
        let mut estimator = Mcl::new(predicted, config).unwrap();
        let estimate = estimator
            .observe(Observations {
                predicted,
                sensors: &sensors,
                reports: &reports,
                map: &map,
            })
            .unwrap();
        assert!(estimate.position.distance(truth.position) < 1e-9);
        assert_eq!(estimate.heading, truth.heading);
    }
}
