//! Sample a tuple or list of tracking sources without estimating pose.

use std::{marker::PhantomData, time::Duration};

use futures_util::future::join_all;
use glam::DVec3;
pub use ozton_math::localization::observation::SensorSample;
use ozton_math::{localization::observation::ObservationLayout, uom::si::f64::Length};
#[cfg(feature = "vex")]
use vexide::time::LowResolutionTime;

#[cfg(feature = "vex")]
use crate::{
    HasDriveWheels, SourceAt,
    inertial::{InertialSensorTracking, Orientation},
    odometric::DifferentialTracking,
};
use crate::{
    Observation, TrackingSystem,
    range::{MAX_RANGES, RangeSensorConfig},
};

#[cfg(feature = "vex")]
mod vex;
#[cfg(feature = "vex")]
pub use vex::{Sensors, SharedSensors};

#[cfg(feature = "vex")]
pub(crate) fn stamp(time: LowResolutionTime) -> Duration {
    time.duration_since(LowResolutionTime::EPOCH)
}

/// A wheel-travel source in meters. A missing observation returns None.
pub trait WheelSource {
    fn read_travel(&self) -> Option<Length>;

    /// Performs any configuration checks before the source is installed.
    fn valid(&self) -> bool {
        true
    }
}

impl WheelSource for () {
    fn read_travel(&self) -> Option<Length> {
        None
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrackingBuildError {
    WheelGeometry,
    RangeLayout,
    ImuStartup,
}

impl std::fmt::Display for TrackingBuildError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::WheelGeometry => f.write_str("invalid tracking-wheel geometry"),
            Self::RangeLayout => write!(f, "too many range sources (maximum {MAX_RANGES})"),
            Self::ImuStartup => f.write_str("IMU startup calibration failed"),
        }
    }
}

impl std::error::Error for TrackingBuildError {}

mod sources;
use sources::{Capture, SourceLayout};
pub use sources::TrackingSources;

/// Live sources and their observations. Pose and estimator state belong to localization.
pub struct Tracking<S, D = ()> {
    sources: S,
    drive: PhantomData<D>,
    range_layout: arrayvec::ArrayVec<(usize, RangeSensorConfig), MAX_RANGES>,
    wheel_mounts: Vec<(usize, DVec3)>,
    observations: Vec<Observation>,
}

impl<S: TrackingSources> TrackingSystem for Tracking<S> {}

#[cfg(feature = "vex")]
impl<S: TrackingSources, const I: usize> TrackingSystem for Tracking<S, DriveSource<I>>
where
    Self: SourceAt<I, Source = DifferentialTracking>,
{
    fn selected_drive(&self) -> Option<&DifferentialTracking> {
        Some(self.source_at())
    }
}

impl<S: TrackingSources> Tracking<S> {
    /// Initialize the tuple or list of sources concurrently before the first sample.
    pub async fn new(mut sources: S) -> Result<Self, TrackingBuildError> {
        const {
            assert!(
                S::DIFFERENTIAL_COUNT <= 1,
                "tracking may contain at most one differential encoder source"
            );
        }
        let mut layout = SourceLayout::default();
        sources.describe(&mut layout)?;
        for result in join_all(sources.initialization()).await {
            result?;
        }
        Ok(Self {
            observations: Vec::with_capacity(S::SOURCE_COUNT),
            drive: PhantomData,
            range_layout: layout.ranges,
            sources,
            wheel_mounts: layout.wheels,
        })
    }

    /// Select the calibrated differential wheel source at assembly time.
    /// Invalid indices and sources of the wrong type fail to compile.
    #[cfg(feature = "vex")]
    pub fn with_drive_source<const I: usize>(self) -> Tracking<S, DriveSource<I>>
    where
        Self: SourceAt<I, Source = DifferentialTracking>,
    {
        Tracking {
            sources: self.sources,
            drive: PhantomData,
            range_layout: self.range_layout,
            wheel_mounts: self.wheel_mounts,
            observations: self.observations,
        }
    }
}

/// Type-level identity of an explicitly selected drive encoder source.
#[cfg(feature = "vex")]
pub struct DriveSource<const I: usize>;

#[cfg(feature = "vex")]
impl<S: TrackingSources, const I: usize> HasDriveWheels for Tracking<S, DriveSource<I>>
where
    Self: SourceAt<I, Source = DifferentialTracking>,
{
    fn calibrated_drive(&self) -> &DifferentialTracking {
        self.source_at()
    }

    fn calibrated_drive_mut(&mut self) -> &mut DifferentialTracking {
        self.source_at_mut()
    }
}

impl<S: TrackingSources, D> Tracking<S, D> {
    pub fn sources(&self) -> &S {
        &self.sources
    }

    pub fn observation_layout(&self) -> ObservationLayout<'_> {
        ObservationLayout {
            wheels: &self.wheel_mounts,
            ranges: &self.range_layout,
        }
    }

    pub fn wheel_mounts(&self) -> &[(usize, DVec3)] {
        &self.wheel_mounts
    }

    #[cfg(feature = "vex")]
    pub fn diagnostic_orientation(&self) -> Option<Orientation> {
        self.sources.diagnostic_imu()?.diagnostic_orientation()
    }

    #[cfg(feature = "vex")]
    pub fn imu_telemetry(&self) -> ([f64; 3], [f64; 3]) {
        self.sources.diagnostic_imu().map_or(
            ([f64::NAN; 3], [f64::NAN; 3]),
            InertialSensorTracking::telemetry,
        )
    }

    #[cfg(feature = "vex")]
    pub fn sample(&mut self, occluded_ranges: u8) -> SensorSample<'_> {
        self.sample_at(stamp(LowResolutionTime::now()), occluded_ranges)
    }

    /// Sample all sources at a timestamp from the caller's monotonic clock.
    pub fn sample_at(&mut self, time: Duration, occluded_ranges: u8) -> SensorSample<'_> {
        self.observations.clear();
        self.sources.capture(&mut Capture::new(
            time,
            occluded_ranges,
            &self.range_layout,
            &mut self.observations,
        ));
        SensorSample {
            time,
            observations: &self.observations,
        }
    }
}

#[cfg(test)]
mod tests {
    use futures_util::FutureExt;

    use super::*;
    use crate::{
        HeadingObservation, Measurement, MeasurementSink, TrackingSource, range::RangeObservation,
    };

    struct Source(bool);

    impl TrackingSource for Source {
        fn range_mount(&self) -> Option<RangeSensorConfig> {
            self.0.then(|| RangeSensorConfig::new(Default::default()))
        }

        fn sample(&mut self, sink: &mut MeasurementSink<'_>) {
            if let Some(index) = sink.range_index {
                assert_eq!(index, 0);
                sink.push(Measurement::Range(RangeObservation {
                    timestamp: sink.time.saturating_sub(Duration::from_millis(1)),
                    distance: ozton_math::qty!(1 m),
                    confidence: 1.0,
                    occluded: sink.range_occluded,
                }));
            } else {
                sink.push(Measurement::Heading(HeadingObservation::Counterclockwise(
                    ozton_math::qty!(15 deg),
                )));
            }
        }
    }

    #[test]
    fn caller_clock_stamps_custom_sources_and_preserves_range_capture_time() {
        let mut tracking = Tracking::new((Source(false), Source(true)))
            .now_or_never()
            .unwrap()
            .unwrap();
        let time = Duration::from_millis(10);
        let sample = tracking.sample_at(time, 1);

        assert_eq!(sample.time, time);
        let [heading, range] = sample.observations else {
            panic!("expected one heading and one range observation");
        };
        assert_eq!((heading.source, heading.time), (0, time));
        assert_eq!((range.source, range.time), (1, Duration::from_millis(9)));
        assert!(matches!(range.measurement, Measurement::Range(range) if range.occluded));
        assert_eq!(tracking.sample_at(time, 0).observations.len(), 2);
    }
}
