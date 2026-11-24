//! Trace the observations and decisions from each localization update.

use ozton_diagnostics::serial::{Frame, Level};
use ozton_math::{
    geometry::Pose, localization::observation::RangeObservation, uom::si::f64::Length,
};

use crate::localizer::{DistanceReset, Localizer, MAX_RANGES, Update};

#[derive(Default)]
pub(super) struct Telemetry {
    sequence: u64,
}

impl Telemetry {
    pub(super) fn record(
        &mut self,
        update: &Update,
        localizer: &Localizer<'_>,
        wheels: &[Option<Length>],
        readings: &[Option<RangeObservation>; MAX_RANGES],
        occluded: u8,
    ) {
        if !ozton_diagnostics::serial::sd_enabled() {
            return;
        }

        self.sequence += 1;
        let frame = Frame {
            sequence: self.sequence,
            time: update.raw.time,
        };
        let reset = match update.reset {
            DistanceReset::Idle => "idle",
            DistanceReset::Waiting => "waiting",
            DistanceReset::Applied { .. } => "applied",
            DistanceReset::Rejected => "rejected",
        };
        frame.record(Level::Trace, "LOCALIZATION", format_args!(
            "estimator={},corrections_enabled={},trust={:?},correction={:?},reset={reset},reset_axes={},range_sensors={},forward_mps={:.4},lateral_mps={:.4},yaw_radps={:.4}",
            localizer.estimator_name(), localizer.corrections_enabled(), localizer.pose_trust(),
            update.correction, update.distance_reset_axes, localizer.range_count(),
            update.forward_velocity, update.lateral_velocity, update.angular_velocity,
        ));
        let [lateral, forward, yaw] = localizer.velocity_uncertainty();
        frame.record(
            Level::Trace,
            "VELOCITY_UNCERTAINTY",
            format_args!("lateral_sigma_mps={lateral:.4},forward_sigma_mps={forward:.4},yaw_sigma_radps={yaw:.4}"),
        );
        for (source, pose) in [
            ("odometry", Some(update.odometry)),
            ("prediction", Some(update.predicted)),
            ("estimate", Some(update.pose)),
            ("candidate", update.candidate),
        ] {
            if let Some(Pose { position, heading }) = pose {
                frame.record(
                    Level::Trace,
                    "LOCALIZATION_POSE",
                    format_args!(
                        "source={source},x_m={:.5},y_m={:.5},heading_rad={:.5}",
                        position.x, position.y, heading.value,
                    ),
                );
            }
        }

        let [left, right] = update.raw.drive.map(meters);
        frame.record(
            Level::Trace,
            "TRACKING_INPUT",
            format_args!(
                "left_m={left:.5},right_m={right:.5},forward_m={:.5},lateral_m={:.5},imu_rad={:.5},occluded_mask={occluded}",
                meters(update.raw.forward),
                meters(update.raw.lateral),
                update.raw.imu.map_or(f64::NAN, |angle| angle.value),
            ),
        );
        for (index, travel) in wheels.iter().copied().enumerate() {
            frame.record(
                Level::Trace,
                "TRACKING_WHEEL",
                format_args!("index={index},travel_m={:.5}", meters(travel)),
            );
        }
        for (index, &reading) in readings.iter().enumerate().take(localizer.range_count()) {
            let report = update.ranges[index];
            let reading = report.and_then(|report| report.observation).or(reading);
            let (verdict, reason, hit, residual) = match report.map(|report| report.constraint) {
                Some(Ok(constraint)) => (
                    "accepted",
                    None,
                    Some(constraint.hit.surface),
                    constraint.residual,
                ),
                Some(Err(rejection)) => ("rejected", Some(rejection), None, f64::NAN),
                None => ("inactive", None, None, f64::NAN),
            };
            let config = localizer.range_config(index);
            let constraint = report.and_then(|report| report.constraint.ok());
            let predicted_m = constraint.map_or(f64::NAN, |c| c.hit.distance);
            let sigma_m = config
                .zip(constraint)
                .zip(report.and_then(|r| r.calibrated_distance))
                .map_or(f64::NAN, |((sensor, c), distance)| {
                    sensor.range_sigma(ozton_math::qty!((distance.value.max(c.hit.distance)) m))
                });
            frame.record(Level::Trace, "LOCALIZATION_RANGE", format_args!(
                "index={index},status={verdict},reason={reason:?},sample_ms={:.0},distance_m={:.4},calibrated_m={:.4},confidence={:.3},occluded={},surface={hit:?},residual_m={residual:.4},predicted_m={predicted_m:.4},range_sigma_m={sigma_m:.4}",
                reading.map_or(f64::NAN, |reading| reading.timestamp.as_secs_f64() * 1000.0),
                meters(reading.map(|reading| reading.distance)),
                meters(report.and_then(|report| report.calibrated_distance).or_else(|| {
                    reading.zip(localizer.range_config(index)).map(|(sample, config)| sample.distance - config.bias)
                })),
                reading.map_or(f64::NAN, |reading| reading.confidence),
                reading.is_some_and(|reading| reading.occluded),
            ));
        }
    }
}

fn meters(value: Option<Length>) -> f64 {
    value.map_or(f64::NAN, |length| length.value)
}
