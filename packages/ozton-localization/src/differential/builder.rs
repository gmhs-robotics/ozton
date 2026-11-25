use ozton_math::localization::observation::{ObservationLayout, RangeLayoutSource};

use super::*;
use crate::{
    Config, DsrConfig, OdometryConfig,
    field::{Boundary, FieldMap},
};

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

/// Builder state requiring statically installed range sources and a usable map.
pub struct RangeRequired;

/// Assemble an estimator from the measured tracking layout, without sampling hardware.
pub struct LocalizationBuilder<B = (), R = ()> {
    field: FieldMap<B>,
    estimator: Estimator,
    odometry: OdometrySettings,
    ranges: RangeUsage,
    dsr: DsrConfig,
    velocity: crate::VelocityConfig,
    range_requirement: R,
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
            range_requirement: (),
        }
    }
}

impl LocalizationBuilder {
    pub fn new() -> Self {
        Self::default()
    }
}

impl<B: Boundary, R> LocalizationBuilder<B, R> {
    #[must_use]
    pub fn with_field<C: Boundary>(
        self,
        field: impl Into<FieldMap<C>>,
    ) -> LocalizationBuilder<C, R> {
        LocalizationBuilder {
            field: field.into(),
            estimator: self.estimator,
            odometry: self.odometry,
            ranges: self.ranges,
            dsr: self.dsr,
            velocity: self.velocity,
            range_requirement: self.range_requirement,
        }
    }

    fn require_ranges(self) -> LocalizationBuilder<B, RangeRequired> {
        LocalizationBuilder {
            field: self.field,
            estimator: self.estimator,
            odometry: self.odometry,
            ranges: self.ranges,
            dsr: self.dsr,
            velocity: self.velocity,
            range_requirement: RangeRequired,
        }
    }

    /// Select a range-capable estimator. For pure odometry, use the default builder.
    /// A runtime estimator selection is conservatively treated as range-capable.
    #[must_use]
    pub fn with_estimator(
        mut self,
        estimator: impl Into<Estimator>,
    ) -> LocalizationBuilder<B, RangeRequired> {
        self.estimator = estimator.into();
        self.require_ranges()
    }

    #[must_use]
    pub fn with_odometry(mut self, odometry: OdometrySettings) -> Self {
        self.odometry = odometry;
        self
    }

    #[must_use]
    pub fn with_ranges(mut self, ranges: RangeUsage) -> LocalizationBuilder<B, RangeRequired> {
        self.ranges = ranges;
        self.require_ranges()
    }

    #[must_use]
    pub fn with_distance_reset(
        mut self,
        config: DsrConfig,
    ) -> LocalizationBuilder<B, RangeRequired> {
        self.dsr = config;
        self.require_ranges()
    }

    #[must_use]
    pub fn with_velocity_estimator(mut self, config: crate::VelocityConfig) -> Self {
        self.velocity = config;
        self
    }

    fn build_inner(
        self,
        layout: ObservationLayout<'_>,
        require_ranges: bool,
    ) -> Option<DifferentialLocalization<B>> {
        let projection = self.field.motion_projection()?;
        let map: Vec<_> = self.field.surfaces().collect();
        let mounts: Vec<_> = layout.ranges.iter().map(|&(_, mount)| mount).collect();
        if require_ranges {
            let eligible =
                mounts
                    .iter()
                    .take(MAX_RANGES)
                    .enumerate()
                    .fold(0u8, |mask, (index, mount)| {
                        mask | (u8::from(mount.with_visible_surfaces(&map).surface_mask != 0)
                            << index)
                    });
            let configured = match self.estimator {
                Estimator::Odometry => self.ranges.dsr | self.ranges.ekf | self.ranges.mcl,
                Estimator::Ekf(_) => self.ranges.ekf,
                Estimator::Mcl(_) => self.ranges.mcl,
            };
            if eligible & configured == 0 {
                return None;
            }
        }
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
            boundary: projection.boundary,
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

impl<B: Boundary> LocalizationBuilder<B> {
    /// Build the default odometry estimator from a raw observation layout.
    pub fn build(self, layout: ObservationLayout<'_>) -> Option<DifferentialLocalization<B>> {
        self.build_inner(layout, false)
    }
}

impl<B: Boundary> LocalizationBuilder<B, RangeRequired> {
    /// Build only with a tracking configuration that supplies ranges. The map and
    /// configured sensor roles must also provide at least one eligible surface.
    ///
    /// ```compile_fail
    /// use ozton_localization::differential::{LocalizationBuilder, RangeUsage};
    /// use ozton_math::localization::observation::ObservationLayout;
    /// let _ = LocalizationBuilder::new()
    ///     .with_ranges(RangeUsage::new())
    ///     .build(&ObservationLayout::default());
    /// ```
    pub fn build(self, tracking: &impl RangeLayoutSource) -> Option<DifferentialLocalization<B>> {
        self.build_inner(tracking.observation_layout(), true)
    }
}

#[cfg(test)]
mod tests {
    use ozton_math::{
        localization::observation::{HasRange, RangeSensorConfig},
        position_xyzh,
    };

    use super::*;
    use crate::field::Field;

    struct Ranges([(usize, RangeSensorConfig); 1]);

    impl HasRange for Ranges {}

    impl RangeLayoutSource for Ranges {
        fn observation_layout(&self) -> ObservationLayout<'_> {
            ObservationLayout {
                wheels: &[],
                ranges: &self.0,
            }
        }
    }

    #[test]
    fn range_builder_requires_a_visible_mapped_surface() {
        let ranges = Ranges([(
            0,
            RangeSensorConfig::at_mount(position_xyzh!(0 in, 0 in, 4 in, 0 deg)),
        )]);
        assert!(
            LocalizationBuilder::new()
                .with_ranges(RangeUsage::new())
                .build(&ranges)
                .is_none()
        );
        assert!(
            LocalizationBuilder::new()
                .with_field(Field::METAL)
                .with_ranges(RangeUsage::new())
                .build(&ranges)
                .is_some()
        );
        let high = Ranges([(
            0,
            RangeSensorConfig::at_mount(position_xyzh!(0 in, 0 in, 20 in, 0 deg)),
        )]);
        assert!(
            LocalizationBuilder::new()
                .with_field(Field::METAL)
                .with_ranges(RangeUsage::new())
                .build(&high)
                .is_none()
        );
    }
}
