use ozton_math::localization::observation::{
    ObservationLayout, ObservationLayoutSource, RangeLayoutSource,
};

use super::*;
use crate::{
    Config, DsrConfig, OdometryConfig, RangeRoles,
    field::{Boundary, FieldMap},
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
    field: FieldMap<B>,
    estimator: Estimator,
    odometry: OdometrySettings,
    dsr: Option<DsrConfig>,
    velocity: crate::VelocityConfig,
    range_requirement: R,
}

impl Default for LocalizationBuilder {
    fn default() -> Self {
        Self {
            field: FieldMap::default(),
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
    #[must_use]
    pub fn with_field<C: Boundary>(
        self,
        field: impl Into<FieldMap<C>>,
    ) -> LocalizationBuilder<C, R> {
        LocalizationBuilder {
            field: field.into(),
            estimator: self.estimator,
            odometry: self.odometry,
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
    pub fn with_distance_reset(
        mut self,
        config: DsrConfig,
    ) -> LocalizationBuilder<B, RangeRequired> {
        self.dsr = Some(config);
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
    ) -> Option<Localization<B>> {
        let projection = self.field.motion_projection()?;
        let map: Vec<_> = self.field.surfaces().collect();
        let mounts: Vec<_> = layout
            .ranges
            .iter()
            .map(|&(_, mount)| self.field.range_config(mount))
            .collect();
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
            boundary: projection.boundary,
            motion_obstacles: projection.fixed,
            movable_obstacles: projection.movable,
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
    fn builder_excludes_movable_faces_only_when_requested() {
        use ozton_math::{geometry::DVec2, qty};

        let ranges = Ranges([(
            0,
            RangeSensorConfig::at_mount(position_xyzh!(0 in, 0 in, 4 in, 0 deg)),
        )]);
        for exclude in [false, true] {
            let mut field = FieldMap::from(Field::METAL);
            assert!(field.add_movable_box(
                DVec2::new(-0.1, 0.4),
                DVec2::new(0.1, 0.6),
                qty!(10 in),
            ));
            if exclude {
                field = field.without_movable_range_landmarks();
            }
            let localization = LocalizationBuilder::new()
                .with_field(field)
                .build(&ranges)
                .unwrap();
            let mask = localization.localizer.range_config(0).unwrap().surface_mask;
            assert_eq!(mask, if exclude { 0b1111 } else { 0xff });
            assert!(localization.movable_obstacles().into_iter().any(|item| item.is_some()));
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
                .with_distance_reset(DsrConfig::default())
                .build(&ranges)
                .is_none()
        );
        assert!(
            LocalizationBuilder::new()
                .with_field(Field::METAL)
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
                .with_field(Field::METAL)
                .with_distance_reset(DsrConfig::default())
                .build(&high)
                .is_none()
        );
    }
}
