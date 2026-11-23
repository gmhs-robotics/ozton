//! Rate-limited diagnostics from the samples already consumed by localization.

use std::time::Duration;

use ozton_diagnostics::serial::Frame;
use ozton_math::{geometry::Pose, localization::range::Rejection, uom::si::f64::Length};
use ozton_tracking::sources::distance::RangeObservation;

use crate::localizer::{Correction, Localizer, MAX_RANGES, RangeReport, Update};

const PERIOD: Duration = Duration::from_millis(250);

#[derive(Default)]
pub(super) struct Telemetry {
    last: Option<Duration>,
    sequence: u64,
    corrections: [u32; 3],
    candidate: Option<Pose>,
    ranges: [Option<RangeReport>; MAX_RANGES],
}

impl Telemetry {
    pub(super) fn record(
        &mut self,
        update: &Update,
        localizer: &Localizer<'_>,
        wheels: &[Option<Length>],
        readings: &[Option<RangeObservation>; MAX_RANGES],
    ) {
        match update.correction {
            Correction::Accepted => self.corrections[0] += 1,
            Correction::Rejected => self.corrections[1] += 1,
            Correction::Shadow => self.corrections[2] += 1,
            Correction::None => {}
        }
        self.candidate = update.candidate.or(self.candidate);
        for (stored, report) in self.ranges.iter_mut().zip(update.ranges) {
            if let Some(report) = report
                && (!matches!(report.constraint, Err(Rejection::Duplicate)) || stored.is_none())
            {
                *stored = Some(report);
            }
        }

        let time = update.raw.time;
        if self
            .last
            .is_some_and(|last| time.saturating_sub(last) < PERIOD)
        {
            return;
        }
        self.last = Some(time);
        self.sequence += 1;
        let frame = Frame {
            sequence: self.sequence,
            time,
        };
        let [accepted, rejected, shadow] = std::mem::take(&mut self.corrections);
        frame.row("LOCALIZATION", format_args!(
            "estimator={},corrections_enabled={},trust={:?},reset={:?},accepted={accepted},rejected={rejected},shadow={shadow},range_sensors={},forward_mps={:.4},lateral_mps={:.4},yaw_radps={:.4}",
            localizer.estimator_name(), localizer.corrections_enabled(), localizer.pose_trust(), update.reset, localizer.range_count(),
            update.forward_velocity, update.lateral_velocity, update.angular_velocity,
        ));
        for (source, pose) in [
            ("odometry", Some(update.odometry)),
            ("prediction", Some(update.predicted)),
            ("estimate", Some(update.pose)),
            ("candidate", self.candidate.take()),
        ] {
            if let Some(pose) = pose {
                frame.row(
                    "LOCALIZATION_POSE",
                    format_args!(
                        "source={source},x_m={:.4},y_m={:.4},heading_rad={:.4}",
                        pose.position.x, pose.position.y, pose.heading.value,
                    ),
                );
            }
        }

        let [left, right] = update.raw.drive.map(meters);
        frame.row(
            "TRACKING_INPUT",
            format_args!(
                "left_m={left:.4},right_m={right:.4},forward_m={:.4},lateral_m={:.4},imu_rad={:.4}",
                meters(update.raw.forward),
                meters(update.raw.lateral),
                update.raw.imu.map_or(f64::NAN, |angle| angle.value),
            ),
        );
        for (index, travel) in wheels.iter().copied().enumerate() {
            frame.row(
                "TRACKING_WHEEL",
                format_args!("index={index},travel_m={:.4}", meters(travel)),
            );
        }
        for (index, stored) in self
            .ranges
            .iter_mut()
            .enumerate()
            .take(localizer.range_count())
        {
            let report = stored.take();
            let reading = report
                .and_then(|report| report.observation)
                .or(readings[index]);
            let (status, hit, residual) = match report.map(|report| report.constraint) {
                Some(Ok(constraint)) => (None, Some(constraint.hit.surface), constraint.residual),
                Some(Err(rejection)) => (Some(rejection), None, f64::NAN),
                None => (None, None, f64::NAN),
            };
            let verdict = if hit.is_some() {
                "accepted"
            } else if status.is_some() {
                "rejected"
            } else {
                "inactive"
            };
            frame.row("LOCALIZATION_RANGE", format_args!(
                "index={index},status={verdict},reason={status:?},distance_m={:.4},confidence={:.3},surface={hit:?},residual_m={residual:.4}",
                reading.map_or(f64::NAN, |reading| reading.distance.value),
                reading.map_or(f64::NAN, |reading| reading.confidence),
            ));
        }
    }
}

fn meters(value: Option<Length>) -> f64 {
    value.map_or(f64::NAN, |length| length.value)
}
