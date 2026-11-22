use super::*;
use crate::{Config, OdometryConfig, field::FieldMap};

/// Which estimator or reset may consume the installed distance sensors.
#[derive(Clone, Copy)]
pub enum DsrUse {
    Dsr,
    Ekf,
    Mcl,
}

#[derive(Clone, Copy)]
pub(super) struct RangeUses {
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
    pub(super) fn apply(self, localizer: &mut Localizer<'_>) {
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
    field: FieldMap,
    estimator: Estimator,
    use_imu: bool,
    drive: Option<DriveEncoderUse>,
    ranges: RangeUses,
}

impl Default for LocalizationBuilder {
    fn default() -> Self {
        Self {
            field: FieldMap::default(),
            estimator: Estimator::Odometry,
            use_imu: true,
            drive: Some(DriveEncoderUse::ForwardAndHeading),
            ranges: RangeUses::default(),
        }
    }
}

impl LocalizationBuilder {
    pub fn new(field: impl Into<FieldMap>) -> Self {
        Self {
            field: field.into(),
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
    pub fn build(self, tracking: &SensorTracking) -> Option<DifferentialLocalization> {
        let map = self.field.surfaces().collect();
        let mounts = tracking
            .range_mounts()
            .iter()
            .copied()
            .map(|mount| self.field.range_config(mount))
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
        )?;
        self.ranges.apply(&mut localizer);
        Some(DifferentialLocalization {
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
