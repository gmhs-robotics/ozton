use ozton_math::localization::observation::ObservationLayout;

use super::*;
use crate::{Config, DsrConfig, OdometryConfig, field::FieldMap};

/// Which estimator or reset may consume the installed distance sensors.
#[derive(Clone, Copy)]
pub enum RangeUse {
    Dsr,
    Ekf,
    Mcl,
}

#[derive(Clone, Copy)]
pub struct RangeUsage {
    dsr: u8,
    ekf: u8,
    mcl: u8,
}

impl Default for RangeUsage {
    fn default() -> Self {
        Self::new()
    }
}

impl RangeUsage {
    pub const fn new() -> Self {
        Self {
            dsr: u8::MAX,
            ekf: u8::MAX,
            mcl: u8::MAX,
        }
    }

    pub(super) fn apply(self, localizer: &mut Localizer<'_>) {
        let filter = match localizer.estimator() {
            Estimator::Odometry => 0,
            Estimator::Ekf(_) => self.ekf,
            Estimator::Mcl(_) => self.mcl,
        };
        localizer.set_range_usage(self.dsr, filter);
    }
    /// Set the roles for the whole distance sensor bank.
    pub fn with_uses(self, uses: impl IntoIterator<Item = RangeUse>) -> Self {
        self.with_mask(u8::MAX, uses)
    }

    /// Set roles for specific distance sensors, indexed in tracking registration order.
    pub fn with_mask(mut self, mask: u8, uses: impl IntoIterator<Item = RangeUse>) -> Self {
        self.dsr &= !mask;
        self.ekf &= !mask;
        self.mcl &= !mask;
        for usage in uses {
            match usage {
                RangeUse::Dsr => self.dsr |= mask,
                RangeUse::Ekf => self.ekf |= mask,
                RangeUse::Mcl => self.mcl |= mask,
            }
        }
        self
    }
}

/// Which available observations contribute to the odometry prediction.
#[derive(Clone, Copy)]
pub struct OdometrySettings {
    use_imu: bool,
    drive: Option<DriveEncoderUse>,
    maximum_gap: Duration,
}

impl Default for OdometrySettings {
    fn default() -> Self {
        Self::new()
    }
}

impl OdometrySettings {
    pub const fn new() -> Self {
        Self {
            use_imu: true,
            drive: Some(DriveEncoderUse::ForwardAndHeading),
            maximum_gap: Duration::from_millis(100),
        }
    }

    #[must_use]
    pub const fn with_imu_heading(mut self, enabled: bool) -> Self {
        self.use_imu = enabled;
        self
    }

    #[must_use]
    pub const fn with_drive_encoders(mut self, usage: DriveEncoderUse) -> Self {
        self.drive = Some(usage);
        self
    }

    #[must_use]
    pub const fn without_drive_encoders(mut self) -> Self {
        self.drive = None;
        self
    }

    #[must_use]
    pub const fn with_maximum_gap(mut self, gap: Duration) -> Self {
        self.maximum_gap = gap;
        self
    }
}

/// Assemble an estimator from the measured tracking layout, without hardware.
pub struct LocalizationBuilder {
    field: FieldMap,
    estimator: Estimator,
    odometry: OdometrySettings,
    ranges: RangeUsage,
    dsr: DsrConfig,
    velocity: crate::VelocityConfig,
}

impl Default for LocalizationBuilder {
    fn default() -> Self {
        Self {
            field: FieldMap::default(),
            estimator: Estimator::Odometry,
            odometry: OdometrySettings::new(),
            ranges: RangeUsage::new(),
            dsr: DsrConfig::default(),
            velocity: crate::VelocityConfig::default(),
        }
    }
}

impl LocalizationBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    #[must_use]
    pub fn with_field(mut self, field: impl Into<FieldMap>) -> Self {
        self.field = field.into();
        self
    }

    #[must_use]
    pub fn with_estimator(mut self, estimator: impl Into<Estimator>) -> Self {
        self.estimator = estimator.into();
        self
    }

    #[must_use]
    pub fn with_odometry(mut self, odometry: OdometrySettings) -> Self {
        self.odometry = odometry;
        self
    }

    #[must_use]
    pub fn with_ranges(mut self, ranges: RangeUsage) -> Self {
        self.ranges = ranges;
        self
    }

    #[must_use]
    pub fn with_distance_reset(mut self, config: DsrConfig) -> Self {
        self.dsr = config;
        self
    }

    #[must_use]
    pub fn with_velocity_estimator(mut self, config: crate::VelocityConfig) -> Self {
        self.velocity = config;
        self
    }

    /// Use the tracker's mount geometry. No sensors are sampled or initialized here.
    pub fn build(self, layout: ObservationLayout<'_>) -> Option<DifferentialLocalization> {
        let projection = self.field.motion_projection()?;
        let map = self.field.surfaces().collect();
        let mounts = layout.ranges.iter().map(|&(_, mount)| mount).collect();
        let mut localizer = Localizer::new_owned(
            Config::new(
                OdometryConfig::for_mounted_wheels(self.odometry.maximum_gap),
                self.estimator,
                false,
            ),
            Pose::default(),
            map,
            mounts,
        )?;
        if !localizer.set_dsr_config(self.dsr) || !localizer.set_velocity_config(self.velocity) {
            return None;
        }
        self.ranges.apply(&mut localizer);
        Some(DifferentialLocalization {
            localizer,
            rectangular_half_extents: projection.bounds,
            motion_obstacles: projection.fixed,
            movable_obstacles: projection.movable,
            wheels: WheelOdometry::new(layout.wheels.iter().map(|&(_, projection)| projection)),
            observations: super::observations::Observations::new(layout),
            drive: self.odometry.drive,
            use_imu: self.odometry.use_imu,
            range_uses: self.ranges,
            occluded: 0,
            last_reset: DistanceReset::Idle,
            feedback: Ok(None),
            #[cfg(feature = "logging")]
            fault: None,
            #[cfg(feature = "logging")]
            telemetry: Default::default(),
        })
    }
}
