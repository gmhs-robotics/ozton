//! Source collections keep devices inline and dispatch periodic samples through their concrete types.

use std::time::Duration;

use glam::DVec3;

use crate::{
    MeasurementSink, Observation, SourceInitialization, TrackingSource, range::RangeSensorConfig,
};

/// Sources ordered by tuple position or array index.
pub trait TrackingSources {
    #[doc(hidden)]
    const SOURCE_COUNT: usize;

    #[doc(hidden)]
    const HAS_SELECTED_DRIVE: bool = false;

    #[doc(hidden)]
    #[cfg(feature = "vex")]
    fn selected_drive(&self) -> Option<&crate::odometric::DifferentialTracking> {
        None
    }

    #[doc(hidden)]
    const DIFFERENTIAL_COUNT: usize;

    #[doc(hidden)]
    const GUARANTEED_RANGE_COUNT: usize;

    #[doc(hidden)]
    fn describe(&self, layout: &mut SourceLayout) -> Result<(), super::TrackingBuildError>;

    #[doc(hidden)]
    #[cfg(feature = "vex")]
    fn diagnostic_imu(&self) -> Option<&crate::inertial::InertialSensorTracking>;

    #[doc(hidden)]
    fn initialization(&mut self) -> Vec<Startup<'_>>;

    #[doc(hidden)]
    fn capture(&mut self, capture: &mut Capture<'_>);
}

pub(super) type Startup<'a> = <SourceInitialization<'a> as IntoIterator>::Item;

impl TrackingSources for () {
    const SOURCE_COUNT: usize = 0;

    const DIFFERENTIAL_COUNT: usize = 0;
    const GUARANTEED_RANGE_COUNT: usize = 0;

    fn describe(&self, _: &mut SourceLayout) -> Result<(), super::TrackingBuildError> {
        Ok(())
    }

    #[cfg(feature = "vex")]
    fn diagnostic_imu(&self) -> Option<&crate::inertial::InertialSensorTracking> {
        None
    }

    fn initialization(&mut self) -> Vec<Startup<'_>> {
        Vec::new()
    }

    fn capture(&mut self, _: &mut Capture<'_>) {}
}

impl<S: TrackingSource, const N: usize> TrackingSources for [S; N] {
    const SOURCE_COUNT: usize = N;

    const DIFFERENTIAL_COUNT: usize = N * S::DIFFERENTIAL_ENCODERS as usize;
    const GUARANTEED_RANGE_COUNT: usize = N * S::RANGE_SLOTS;

    fn describe(&self, layout: &mut SourceLayout) -> Result<(), super::TrackingBuildError> {
        for source in self {
            layout.register(source)?;
        }
        Ok(())
    }

    #[cfg(feature = "vex")]
    fn diagnostic_imu(&self) -> Option<&crate::inertial::InertialSensorTracking> {
        self.iter().find_map(TrackingSource::inertial)
    }

    fn initialization(&mut self) -> Vec<Startup<'_>> {
        self.iter_mut()
            .filter_map(TrackingSource::initialize)
            .collect()
    }

    fn capture(&mut self, capture: &mut Capture<'_>) {
        for source in self {
            capture.sample(source);
        }
    }
}

/// Startup geometry collected through concrete source types.
#[doc(hidden)]
#[derive(Default)]
pub struct SourceLayout {
    next_source: usize,
    pub(super) ranges: arrayvec::ArrayVec<(usize, RangeSensorConfig), { crate::range::MAX_RANGES }>,
    pub(super) wheels: Vec<(usize, DVec3)>,
}

impl SourceLayout {
    pub fn register(
        &mut self,
        source: &impl TrackingSource,
    ) -> Result<(), super::TrackingBuildError> {
        self.register_config(source, source.range_mount())
    }

    pub(super) fn register_range(
        &mut self,
        source: &impl crate::range::RangeSource,
    ) -> Result<(), super::TrackingBuildError> {
        self.register_config(source, Some(source.range_config()))
    }

    fn register_config(
        &mut self,
        source: &impl TrackingSource,
        mount: Option<RangeSensorConfig>,
    ) -> Result<(), super::TrackingBuildError> {
        use super::TrackingBuildError;
        let index = self.next_source;
        self.next_source += 1;
        if let Some(mount) = mount {
            self.ranges
                .try_push((index, mount))
                .map_err(|_| TrackingBuildError::RangeLayout)?;
        }
        if let Some(projection) = source.wheel_projection() {
            if !projection.is_finite() {
                return Err(TrackingBuildError::WheelGeometry);
            }
            self.wheels.push((index, projection));
        }
        Ok(())
    }
}

#[doc(hidden)]
pub struct Capture<'a> {
    time: Duration,
    occluded: u8,
    next_range: usize,
    next_source: usize,
    range_layout: &'a [(usize, RangeSensorConfig)],
    observations: &'a mut Vec<Observation>,
}

impl<'a> Capture<'a> {
    pub(super) fn new(
        time: Duration,
        occluded: u8,
        range_layout: &'a [(usize, RangeSensorConfig)],
        observations: &'a mut Vec<Observation>,
    ) -> Self {
        Self {
            time,
            occluded,
            next_range: 0,
            next_source: 0,
            range_layout,
            observations,
        }
    }

    pub fn sample(&mut self, source: &mut impl TrackingSource) {
        let index = self.next_source;
        self.next_source += 1;
        let range_index = self
            .range_layout
            .get(self.next_range)
            .is_some_and(|(source, _)| *source == index)
            .then(|| {
                let range = self.next_range;
                self.next_range += 1;
                range
            });
        source.sample(&mut MeasurementSink {
            time: self.time,
            source: index,
            range_index,
            range_occluded: range_index.is_some_and(|range| self.occluded & (1 << range) != 0),
            observations: self.observations,
        });
    }
}
