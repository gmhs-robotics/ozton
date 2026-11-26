use ozton_math::localization::observation::{
    ObservationLayout, ObservationLayoutSource, RangeLayoutSource,
};

use super::*;
use crate::{
    Config, DsrConfig, OdometryConfig, RangeRoles,
    field::{Boundary, FieldBoundary, SurfaceMap},
};

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
#[must_use]
pub struct LocalizationBuilder<B = (), R = ()> {
    map: SurfaceMap,
    boundary: B,
    estimator: Estimator,
    odometry: OdometrySettings,
    dsr: Option<DsrConfig>,
    velocity: crate::VelocityConfig,
    range_requirement: R,
}

impl Default for LocalizationBuilder {
    fn default() -> Self {
        Self {
            map: SurfaceMap::new(),
            boundary: (),
            estimator: Estimator::Odometry,
            odometry: OdometrySettings::new(),
            dsr: None,
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
    /// Supply the surveyed optical map. No motion policy is attached.
    pub fn with_surface_map(mut self, map: SurfaceMap) -> Self {
        self.map = map;
        self
    }

    /// Optional measured physical bounds for corner/flush-wall registration.
    /// Does not add optical faces or configure motion containment.
    pub fn with_contact_bounds(
        self,
        boundary: FieldBoundary,
    ) -> LocalizationBuilder<FieldBoundary, R> {
        LocalizationBuilder {
            map: self.map,
            boundary,
            estimator: self.estimator,
            odometry: self.odometry,
            dsr: self.dsr,
            velocity: self.velocity,
            range_requirement: self.range_requirement,
        }
    }

    fn require_ranges(self) -> LocalizationBuilder<B, RangeRequired> {
        LocalizationBuilder {
            map: self.map,
            boundary: self.boundary,
            estimator: self.estimator,
            odometry: self.odometry,
            dsr: self.dsr,
            velocity: self.velocity,
            range_requirement: RangeRequired,
        }
    }

    /// Select a range-capable estimator. For pure odometry, use the default builder.
    /// A runtime estimator selection is conservatively treated as range-capable.
    pub fn with_estimator(
        mut self,
        estimator: impl Into<Estimator>,
    ) -> LocalizationBuilder<B, RangeRequired> {
        self.estimator = estimator.into();
        self.require_ranges()
    }

    pub fn with_odometry(mut self, odometry: OdometrySettings) -> Self {
        self.odometry = odometry;
        self
    }

    pub fn with_distance_reset(
        mut self,
        config: DsrConfig,
    ) -> LocalizationBuilder<B, RangeRequired> {
        self.dsr = Some(config);
        self.require_ranges()
    }

    pub fn with_velocity_estimator(mut self, config: crate::VelocityConfig) -> Self {
        self.velocity = config;
        self
    }

    fn build_inner(
        self,
        layout: ObservationLayout<'_>,
        require_ranges: bool,
    ) -> Option<Localization<B>> {
        let map: Vec<_> = self.map.faces().collect();
        let mounts: Vec<_> = layout.ranges.iter().map(|&(_, mount)| mount).collect();
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
        if require_ranges {
            let roles = match self.estimator {
                Estimator::Odometry => RangeRoles::ALL,
                Estimator::Ekf(_) => RangeRoles::EKF,
                Estimator::Mcl(_) => RangeRoles::MCL,
            };
            if !localizer.has_mapped_ranges(roles) {
                return None;
            }
        }
        if let Some(dsr) = self.dsr
            && (!localizer.has_mapped_ranges(RangeRoles::DSR) || !localizer.set_dsr_config(dsr))
        {
            return None;
        }
        if !localizer.set_velocity_config(self.velocity) {
            return None;
        }
        if matches!(self.estimator, Estimator::Odometry) {
            localizer.disable_range_inspection();
        }
        Some(Localization {
            localizer,
            boundary: self.boundary,
            wheels: WheelOdometry::new(layout.wheels.iter().map(|&(_, projection)| projection)),
            observations: super::observations::Observations::new(layout),
            drive: self.odometry.drive,
            use_imu: self.odometry.use_imu,
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
    /// Build odometry from the configured tracking sources or a supplied layout.
    pub fn build(self, tracking: &impl ObservationLayoutSource) -> Option<Localization<B>> {
        self.build_inner(tracking.observation_layout(), false)
    }
}

impl<B: Boundary> LocalizationBuilder<B, RangeRequired> {
    /// Build only with a tracking configuration that supplies ranges. The map and
    /// configured sensor roles must also provide at least one eligible surface.
    ///
    /// ```compile_fail
    /// use ozton_localization::{DsrConfig, LocalizationBuilder};
    /// use ozton_math::localization::observation::ObservationLayout;
    /// let _ = LocalizationBuilder::new()
    ///     .with_distance_reset(DsrConfig::default())
    ///     .build(&ObservationLayout::default());
    /// ```
    pub fn build(self, tracking: &impl RangeLayoutSource) -> Option<Localization<B>> {
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

    impl ObservationLayoutSource for Ranges {
        fn observation_layout(&self) -> ObservationLayout<'_> {
            ObservationLayout {
                wheels: &[],
                ranges: &self.0,
            }
        }
    }

    #[test]
    fn motion_only_geometry_does_not_change_localization_mask() {
        use ozton_math::{geometry::DVec2, qty};

        use crate::field::{MotionMap, Surface};

        let ranges = Ranges([(
            0,
            RangeSensorConfig::at_mount(position_xyzh!(0 in, 0 in, 4 in, 0 deg)),
        )]);
        let mut map = Field::METAL.surface_map().unwrap();
        let obstacle =
            Surface::rectangle(DVec2::new(-0.1, 0.4), DVec2::new(0.1, 0.6), qty!(10 in)).unwrap();
        let mut motion = MotionMap::from_localization(&map);
        motion.add(obstacle);
        let before = LocalizationBuilder::new()
            .with_surface_map(map.clone())
            .build(&ranges)
            .unwrap();
        assert_eq!(
            before.localizer.range_config(0).unwrap().surface_mask,
            0b1111
        );
        assert!(map.add(obstacle));
        let after = LocalizationBuilder::new()
            .with_surface_map(map)
            .build(&ranges)
            .unwrap();
        assert_eq!(after.localizer.range_config(0).unwrap().surface_mask, 0xff);
    }

    #[test]
    fn range_builder_requires_a_visible_mapped_surface() {
        let ranges = Ranges([(
            0,
            RangeSensorConfig::at_mount(position_xyzh!(0 in, 0 in, 4 in, 0 deg)),
        )]);
        assert!(
            LocalizationBuilder::new()
                .with_distance_reset(DsrConfig::default())
                .build(&ranges)
                .is_none()
        );
        assert!(
            LocalizationBuilder::new()
                .with_surface_map(Field::METAL.surface_map().unwrap())
                .with_distance_reset(DsrConfig::default())
                .build(&ranges)
                .is_some()
        );
        let high = Ranges([(
            0,
            RangeSensorConfig::at_mount(position_xyzh!(0 in, 0 in, 20 in, 0 deg)),
        )]);
        assert!(
            LocalizationBuilder::new()
                .with_surface_map(Field::METAL.surface_map().unwrap())
                .with_distance_reset(DsrConfig::default())
                .build(&high)
                .is_none()
        );
    }
}
