//! Convert timestamped tracking observations into differential robot pose and motion.
use std::time::Duration;

#[cfg(feature = "logging")]
use ozton_diagnostics::log_event;
use ozton_diagnostics::log_row;
use ozton_math::{
    geometry::{DVec2, Pose, compass_heading},
    uom::si::{
        angular_velocity::radian_per_second,
        f64::{Angle, AngularVelocity, Length, Velocity},
        length::meter,
        velocity::meter_per_second,
    },
};
pub use ozton_tracking::odometric::wheel::DriveWheels as DriveEncoderUse;
use ozton_tracking::{
    Tracking, TrackingSystem, TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity,
    sampling::SensorSample,
};

use crate::{
    Feedback, RangeFeedback,
    localizer::{DistanceReset, Estimator, Fault, Localizer, MAX_RANGES, PoseTrust},
};

mod builder;
mod observations;
#[cfg(feature = "logging")]
mod telemetry;
use builder::RangeUses;
pub use builder::{DsrUse, LocalizationBuilder};

/// Pose estimator state. Accepts observations without owning or reading hardware.
pub struct DifferentialLocalization {
    localizer: Localizer<'static>,
    rectangular_half_extents: Option<DVec2>,
    wheels: WheelOdometry,
    observations: observations::Observations,
    drive: Option<DriveEncoderUse>,
    use_imu: bool,
    range_uses: RangeUses,
    imu_heading_offset: Option<Angle>,
    occluded: u8,
    last_reset: DistanceReset,
    #[cfg(feature = "logging")]
    fault: Option<Fault>,
    #[cfg(feature = "logging")]
    telemetry: telemetry::Telemetry,
}

impl DifferentialLocalization {
    /// Half extents of the configured field if it has only a rectangular perimeter.
    pub fn rectangular_half_extents(&self) -> Option<DVec2> {
        self.rectangular_half_extents
    }

    pub fn set_competition_mode(&mut self, mode: LocalizationMode) {
        self.localizer.set_corrections_enabled(
            mode == LocalizationMode::Autonomous
                || !matches!(self.localizer.estimator(), Estimator::Mcl(_)),
        );
        if mode == LocalizationMode::Disabled {
            self.pause_disabled_odometry();
        }
    }

    pub fn pause_disabled_odometry(&mut self) {
        self.localizer.invalidate_pose();
        self.wheels.observe(&[]);
    }

    pub fn register_start(&mut self, pose: Pose, imu: Option<Angle>) -> bool {
        if !self.set_pose(pose) {
            return false;
        }
        self.imu_heading_offset = imu
            .filter(|heading| heading.value.is_finite())
            .map(|heading| pose.heading - compass_heading(heading));
        log_row!(
            Debug,
            "START_POSE",
            "x_m={:.5},y_m={:.5},heading_rad={:.5},imu_offset_rad={:.5}",
            pose.position.x,
            pose.position.y,
            pose.heading.value,
            self.imu_heading_offset
                .map_or(f64::NAN, |angle| angle.value)
        );
        true
    }

    pub fn field_heading(&self, imu: Option<Angle>) -> Option<Angle> {
        let heading = imu.filter(|heading| heading.value.is_finite())?;
        Some(compass_heading(heading) + self.imu_heading_offset?)
    }

    pub fn pose(&self) -> Pose {
        self.localizer.pose()
    }

    pub fn pose_trusted(&self) -> bool {
        self.localizer.pose_trust() == PoseTrust::Trusted
    }

    pub fn set_pose(&mut self, pose: Pose) -> bool {
        if !self.localizer.set_pose(pose) {
            return false;
        }
        self.last_reset = DistanceReset::Idle;
        self.wheels.observe(&[]);
        true
    }

    pub fn set_estimator(&mut self, estimator: Estimator, shadow: bool) -> bool {
        if !self.localizer.set_estimator(estimator, shadow) {
            return false;
        }
        self.range_uses.apply(&mut self.localizer);
        true
    }

    pub fn request_distance_reset(&mut self) {
        self.last_reset = DistanceReset::Idle;
        self.localizer.request_distance_reset();
    }

    pub fn cancel_distance_reset(&mut self) {
        self.localizer.cancel_distance_reset();
    }

    pub fn distance_reset_axes(&self) -> u8 {
        match self.last_reset {
            DistanceReset::Applied { axes } => axes,
            _ => 0,
        }
    }

    pub fn set_occluded(&mut self, mask: u8) {
        self.occluded = mask;
    }

    pub fn occluded(&self) -> u8 {
        self.occluded
    }

    pub fn uses_drive_encoders(&self) -> bool {
        self.drive.is_some()
    }

    pub fn update(
        &mut self,
        sample: SensorSample<'_>,
        track_width: Option<Length>,
    ) -> Result<Option<Feedback>, Fault> {
        let (snapshot, wheels, ranges) = self.observations.read(sample);
        self.wheels.observe(wheels);
        let update =
            match self
                .localizer
                .update_with_odometry(snapshot, &ranges, |before, after, _, _| {
                    Ok(self
                        .wheels
                        .delta(before, after, self.drive, track_width, self.use_imu))
                }) {
                Ok(update) => {
                    #[cfg(feature = "logging")]
                    if self.fault.take().is_some() {
                        log_event!(Info, "tracking", "Localization feedback recovered");
                    }
                    update
                }
                Err(Fault::Initializing) => {
                    log_row!(
                        Trace,
                        "TRACKING_UPDATE",
                        "status=initializing,snapshot_ms={}",
                        snapshot.time.as_millis()
                    );
                    return Ok(None);
                }
                Err(fault) => {
                    #[cfg(feature = "logging")]
                    if self.fault.replace(fault) != Some(fault) {
                        let reason = match fault {
                            Fault::Initializing => "Waiting for a second tracking observation",
                            Fault::Time => "Tracking timestamp repeated or exceeded maximum gap",
                            Fault::Forward => "Forward travel unavailable",
                            Fault::Heading => "Heading unavailable",
                            Fault::Lateral => "Lateral travel unavailable",
                            Fault::InvalidEstimate => "Estimator produced an invalid pose",
                        };
                        log_event!(
                            Warn,
                            "tracking",
                            "{reason} at sample {} ms",
                            snapshot.time.as_millis()
                        );
                    }
                    log_row!(
                        Trace,
                        "TRACKING_UPDATE",
                        "status=fault,fault={fault:?},snapshot_ms={}",
                        snapshot.time.as_millis()
                    );
                    return Err(fault);
                }
            };

        #[cfg(feature = "logging")]
        self.telemetry
            .record(&update, &self.localizer, wheels, &ranges, self.occluded);

        if update.reset != DistanceReset::Idle {
            self.last_reset = update.reset;
        }
        let mut relative_ranges = [None; MAX_RANGES];
        for (index, reading) in ranges.into_iter().enumerate() {
            if let Some(reading) = reading
                && let Some(config) = self.localizer.range_config(index)
                && let Ok(distance) = config.relative_distance(reading, snapshot.time)
            {
                relative_ranges[index] = Some(RangeFeedback {
                    distance,
                    timestamp: reading.timestamp,
                });
            }
        }
        Ok(Some(Feedback {
            time: snapshot.time,
            pose: update.pose,
            forward_travel: Length::new::<meter>(self.localizer.forward_travel()),
            forward_velocity: Velocity::new::<meter_per_second>(update.forward_velocity),
            lateral_velocity: Velocity::new::<meter_per_second>(update.lateral_velocity),
            angular_velocity: AngularVelocity::new::<radian_per_second>(update.angular_velocity),
            ranges: relative_ranges,
            reset_accepted: update.distance_reset,
        }))
    }
}

impl TrackingSystem for DifferentialLocalization {}
impl TracksPosition for DifferentialLocalization {
    fn position(&self) -> glam::DVec2 {
        self.localizer.position()
    }
}
impl TracksHeading for DifferentialLocalization {
    fn heading(&self) -> Angle {
        self.localizer.heading()
    }
}
impl TracksForwardTravel for DifferentialLocalization {
    fn forward_travel(&self) -> f64 {
        self.localizer.forward_travel()
    }
}
impl TracksVelocity for DifferentialLocalization {
    fn linear_velocity(&self) -> f64 {
        self.localizer.linear_velocity()
    }
    fn angular_velocity(&self) -> f64 {
        self.localizer.angular_velocity()
    }
}

use ozton_tracking::{
    odometric::wheel::WheelOdometry,
    sampling::{LocalizationLifecycle, LocalizationMode},
};

impl LocalizationLifecycle<Tracking> for DifferentialLocalization {
    fn mode_changed(&mut self, mode: LocalizationMode) {
        self.set_competition_mode(mode);
    }

    fn cancel_pending(&mut self) {
        self.cancel_distance_reset();
    }

    #[cfg(feature = "vex")]
    fn poll(&mut self, tracking: &mut Tracking) {
        let _ = update_localization(self, tracking);
    }
}

#[cfg(feature = "vex")]
pub fn update_localization(
    localization: &mut DifferentialLocalization,
    tracking: &mut Tracking,
) -> Result<Option<Feedback>, Fault> {
    let width = tracking.drive_wheels().map(|source| source.track_width());
    let sample = tracking.sample(localization.occluded());
    localization.update(sample, width)
}
