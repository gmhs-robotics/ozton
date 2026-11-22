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
pub use ozton_tracking::localizer::wheels::DriveWheels as DriveEncoderUse;
use ozton_tracking::{
    TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity,
    field::Field,
    localizer::{
        Config, DistanceReset, Estimator, Fault, Localizer, MAX_RANGES, OdometryConfig, PoseTrust,
        wheels::WheelOdometry,
    },
    sampling::{SensorSample, SensorTracking},
};

use super::{DriveMotionFault, Feedback, RangeFeedback};

/// Which estimator or reset may consume the installed distance sensors.
#[derive(Clone, Copy)]
pub enum DsrUse {
    Dsr,
    Ekf,
    Mcl,
}

#[derive(Clone, Copy)]
struct RangeUses {
    dsr: u8,
    ekf: u8,
    mcl: u8,
}

impl Default for RangeUses {
    fn default() -> Self {
        Self {
            dsr: u8::MAX,
            ekf: u8::MAX,
            mcl: u8::MAX,
        }
    }
}

impl RangeUses {
    fn apply(self, localizer: &mut Localizer<'_>) {
        let filter = match localizer.estimator() {
            Estimator::Odometry => 0,
            Estimator::Ekf(_) => self.ekf,
            Estimator::Mcl(_) => self.mcl,
        };
        localizer.set_range_usage(self.dsr, filter);
    }
}

/// Choose how the available observations contribute to localization.
pub struct LocalizationBuilder {
    field: Option<Field>,
    estimator: Estimator,
    use_imu: bool,
    drive: Option<DriveEncoderUse>,
    ranges: RangeUses,
}

impl Default for LocalizationBuilder {
    fn default() -> Self {
        Self {
            field: None,
            estimator: Estimator::Odometry,
            use_imu: true,
            drive: Some(DriveEncoderUse::ForwardAndHeading),
            ranges: RangeUses::default(),
        }
    }
}

impl LocalizationBuilder {
    pub fn new(field: Field) -> Self {
        Self {
            field: Some(field),
            ..Self::default()
        }
    }

    pub fn with_estimator(mut self, estimator: Estimator) -> Self {
        self.estimator = estimator;
        self
    }

    pub fn with_imu_heading(mut self, enabled: bool) -> Self {
        self.use_imu = enabled;
        self
    }

    pub fn with_drive_encoders(mut self, usage: DriveEncoderUse) -> Self {
        self.drive = Some(usage);
        self
    }

    pub fn without_drive_encoders(mut self) -> Self {
        self.drive = None;
        self
    }

    /// Set the roles for the whole distance sensor bank.
    pub fn with_range_usage(self, uses: impl IntoIterator<Item = DsrUse>) -> Self {
        self.with_range_mask(u8::MAX, uses)
    }

    /// Set roles for specific distance sensors, indexed in tracking registration order.
    pub fn with_range_mask(mut self, mask: u8, uses: impl IntoIterator<Item = DsrUse>) -> Self {
        self.ranges.dsr &= !mask;
        self.ranges.ekf &= !mask;
        self.ranges.mcl &= !mask;
        for usage in uses {
            match usage {
                DsrUse::Dsr => self.ranges.dsr |= mask,
                DsrUse::Ekf => self.ranges.ekf |= mask,
                DsrUse::Mcl => self.ranges.mcl |= mask,
            }
        }
        self
    }

    /// Use the tracker's mount geometry. No sensors are sampled or initialized here.
    pub fn build(
        self,
        tracking: &SensorTracking,
    ) -> Result<DifferentialLocalization, DriveMotionFault> {
        let map = self
            .field
            .map_or_else(Vec::new, |field| field.surfaces().to_vec());
        let mounts = tracking
            .range_mounts()
            .iter()
            .copied()
            .map(|mut mount| {
                if let Some(field) = self.field {
                    return field.range_config(mount);
                }
                mount.surface_mask = 0;
                mount
            })
            .collect();
        let mut localizer = Localizer::new_owned(
            Config::new(
                OdometryConfig::for_mounted_wheels(Duration::from_millis(100)),
                self.estimator,
                false,
            ),
            Pose::default(),
            map,
            mounts,
        )
        .ok_or(DriveMotionFault::InvalidCommand)?;
        self.ranges.apply(&mut localizer);
        Ok(DifferentialLocalization {
            localizer,
            wheels: WheelOdometry::new(tracking.wheel_projections()),
            drive: self.drive,
            use_imu: self.use_imu,
            range_uses: self.ranges,
            imu_heading_offset: None,
            occluded: 0,
            last_reset: DistanceReset::Idle,
            fault: None,
        })
    }
}

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
    pub fn pause_disabled_odometry(&mut self) {
        self.localizer.invalidate_pose();
        self.wheels.observe(&[]);
    }

    pub(super) fn register_start(
        &mut self,
        pose: Pose,
        imu: Option<Angle>,
    ) -> Result<(), DriveMotionFault> {
        if !self.set_pose(pose) {
            return Err(DriveMotionFault::InvalidCommand);
        }
        self.imu_heading_offset = imu
            .filter(|heading| heading.value.is_finite())
            .map(|heading| Angle::new::<radian>(pose.heading) - compass_heading(heading));
        Ok(())
    }

    pub(super) fn field_heading(&self, imu: Option<Angle>) -> Option<Angle> {
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

    pub(super) fn occluded(&self) -> u8 {
        self.occluded
    }

    pub(super) fn uses_drive_encoders(&self) -> bool {
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
                        ozton_runtime::diagnostic_event!("tracking_recovered");
                    }
                    update
                }
                Err(Fault::Initializing) => return Ok(None),
                Err(fault) => {
                    if self.fault.replace(fault) != Some(fault) {
                        ozton_runtime::diagnostic_event!("tracking_fault={fault:?}");
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
