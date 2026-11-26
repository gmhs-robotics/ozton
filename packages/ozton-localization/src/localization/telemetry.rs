//! Trace the observations and decisions from each localization update.

use std::time::Duration;

use ozton_diagnostics::serial::{Frame, Level};
use ozton_math::{
    geometry::Pose, localization::observation::RangeObservation, uom::si::f64::Length,
};

use crate::localizer::{DistanceReset, Localizer, MAX_RANGES, MotionObservations, Update};

const DETAIL_INTERVAL: Duration = Duration::from_millis(50);
const WHEEL_INTERVAL: Duration = Duration::from_millis(20);

#[derive(Default)]
pub(super) struct Telemetry {
    sequence: u64,
    last_detail: Option<Duration>,
    last_wheels: Option<Duration>,
    last_reset: Option<&'static str>,
}

impl Telemetry {
    pub(super) fn record(
        &mut self,
        update: &Update,
        localizer: &Localizer<'_>,
        wheels: &[Option<Length>],
        readings: &[Option<RangeObservation>; MAX_RANGES],
        occluded: u8,
        rates: MotionObservations,
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
        #[cfg(all(feature = "logging", target_os = "vexos"))]
        let pressure = ozton_diagnostics::sd::under_pressure();
        #[cfg(not(all(feature = "logging", target_os = "vexos")))]
        let pressure = false;
        let detail_interval = if pressure {
            Duration::from_millis(200)
        } else {
            DETAIL_INTERVAL
        };
        let detailed = self.last_reset != Some(reset)
            || self.last_detail.is_none_or(|last| {
                frame.time < last || frame.time.saturating_sub(last) >= detail_interval
            });
        self.last_reset = Some(reset);
        if detailed {
            self.last_detail = Some(frame.time);
        }
        if detailed {
            let [lateral, forward, yaw] = localizer.velocity_uncertainty();
            let covariance = localizer.position_covariance();
            frame.record(
                Level::Trace,
                "VELOCITY_UNCERTAINTY",
                format_args!("lateral_sigma_mps={lateral:.4},forward_sigma_mps={forward:.4},yaw_sigma_radps={yaw:.4},cov_xx_m2={:.7},cov_xy_m2={:.7},cov_yy_m2={:.7}",
                    covariance.map_or(f64::NAN, |p| p.x_axis.x),
                    covariance.map_or(f64::NAN, |p| p.x_axis.y),
                    covariance.map_or(f64::NAN, |p| p.y_axis.y)),
            );
        }
        for (source, pose) in [
            ("odometry", Some(update.odometry)),
            ("prediction", Some(update.predicted)),
            ("estimate", Some(update.pose)),
            ("candidate", update.candidate),
        ] {
            if !detailed && source != "estimate" {
                continue;
            }
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
                "left_m={left:.5},right_m={right:.5},imu_rad={:.5},occluded_mask={occluded},raw_forward_mps={:.5},raw_yaw_radps={:.5},accel_right_mps2={:.5},accel_forward_mps2={:.5}",
                update.raw.imu.map_or(f64::NAN, |angle| angle.value),
                rates.forward_rate.unwrap_or(f64::NAN),
                rates.yaw_rate.unwrap_or(f64::NAN),
                rates.acceleration.map_or(f64::NAN, |a| a.x),
                rates.acceleration.map_or(f64::NAN, |a| a.y),
            ),
        );
        // Keep raw pod data independent of lower-priority range diagnostics.
        let wheel_interval = if pressure {
            DETAIL_INTERVAL
        } else {
            WHEEL_INTERVAL
        };
        if self.last_wheels.is_none_or(|last| {
            frame.time < last || frame.time.saturating_sub(last) >= wheel_interval
        }) {
            self.last_wheels = Some(frame.time);
            for (index, travel) in wheels.iter().copied().enumerate() {
                frame.record(
                    Level::Trace,
                    "TRACKING_WHEEL",
                    format_args!("index={index},travel_m={:.5}", meters(travel)),
                );
            }
        }
        if !detailed {
            return;
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
