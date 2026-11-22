//! Convert timestamped tracking observations into differential robot pose and motion.
use std::time::Duration;

use ozton_math::{
    geometry::{Pose, compass_heading},
    uom::si::{
        angle::radian,
        angular_velocity::radian_per_second,
        f64::{Angle, AngularVelocity, Length, Velocity},
        length::meter,
        velocity::meter_per_second,
    },
};
pub use ozton_tracking::sources::wheels::DriveWheels as DriveEncoderUse;
use ozton_tracking::{
    TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity,
    sampling::{SensorSample, SensorTracking},
};

use crate::{
    Feedback, RangeFeedback,
    localizer::{
        DistanceReset, Estimator, Fault, Localizer, MAX_RANGES, PoseTrust, wheels::WheelOdometry,
    },
};

mod builder;
use builder::RangeUses;
pub use builder::{DsrUse, LocalizationBuilder};

/// Pose estimator state. Accepts observations without owning or reading hardware.
pub struct DifferentialLocalization {
    localizer: Localizer<'static>,
    wheels: WheelOdometry,
    drive: Option<DriveEncoderUse>,
    use_imu: bool,
    range_uses: RangeUses,
    imu_heading_offset: Option<Angle>,
    occluded: u8,
    last_reset: DistanceReset,
    fault: Option<Fault>,
}

impl DifferentialLocalization {
    pub fn set_competition_mode(&mut self, mode: vexide::competition::CompetitionMode) {
        use vexide::competition::CompetitionMode;
        self.localizer.set_corrections_enabled(
            mode == CompetitionMode::Autonomous
                || !matches!(self.localizer.estimator(), Estimator::Mcl(_)),
        );
        if mode == CompetitionMode::Disabled {
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
            .map(|heading| Angle::new::<radian>(pose.heading) - compass_heading(heading));
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
        let SensorSample {
            snapshot,
            wheels,
            ranges,
        } = sample;
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
                    if self.fault.take().is_some() {
                        tracking_event(format_args!("tracking_recovered"));
                    }
                    update
                }
                Err(Fault::Initializing) => return Ok(None),
                Err(fault) => {
                    if self.fault.replace(fault) != Some(fault) {
                        tracking_event(format_args!("tracking_fault={fault:?}"));
                    }
                    return Err(fault);
                }
            };

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

impl ozton_tracking::Tracking for DifferentialLocalization {}
impl TracksPosition for DifferentialLocalization {
    fn position(&self) -> glam::DVec2 {
        self.localizer.position()
    }
}
impl TracksHeading for DifferentialLocalization {
    fn heading(&self) -> vexide::math::Angle {
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

use ozton_tracking::sampling::LocalizationLifecycle;
use vexide::competition::CompetitionMode;

impl LocalizationLifecycle<SensorTracking> for DifferentialLocalization {
    fn mode_changed(&mut self, mode: CompetitionMode) {
        self.set_competition_mode(mode);
    }

    fn cancel_pending(&mut self) {
        self.cancel_distance_reset();
    }

    fn uses_drive_encoders(&self, tracking: &SensorTracking) -> bool {
        tracking.uses_drive_encoders() && self.uses_drive_encoders()
    }

    fn poll(
        &mut self,
        tracking: &mut SensorTracking,
        positions: [Option<Length>; 2],
        width: Option<Length>,
    ) {
        let _ = update_localization(self, tracking, positions, width);
    }
}

pub fn update_localization(
    localization: &mut DifferentialLocalization,
    tracking: &mut SensorTracking,
    positions: [Option<Length>; 2],
    width: Option<Length>,
) -> Result<Option<Feedback>, Fault> {
    let sample = tracking.capture(positions, localization.occluded());
    localization.update(sample, width)
}

fn tracking_event(fields: std::fmt::Arguments<'_>) {
    #[cfg(feature = "logging")]
    ozton_diagnostics::serial::event(vexide::time::user_uptime(), fields);
    #[cfg(not(feature = "logging"))]
    let _ = fields;
}
