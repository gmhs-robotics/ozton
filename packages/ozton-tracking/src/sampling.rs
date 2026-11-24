//! Sample a tuple or list of tracking sources without estimating pose.

use std::time::Duration;

use futures_util::future::join_all;
use glam::DVec3;
pub use ozton_math::localization::observation::SensorSample;
use ozton_math::{localization::observation::ObservationLayout, uom::si::f64::Length};
#[cfg(feature = "vex")]
use vexide::time::LowResolutionTime;

use crate::{
    Observation, TrackingSystem,
    range::{MAX_RANGES, RangeSensorConfig},
};
#[cfg(feature = "vex")]
use crate::{
    inertial::{InertialSensorTracking, Orientation},
    odometric::DifferentialTracking,
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
    DuplicateDriveEncoders,
}

impl std::fmt::Display for TrackingBuildError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::WheelGeometry => f.write_str("invalid tracking-wheel geometry"),
            Self::RangeLayout => write!(f, "too many range sources (maximum {MAX_RANGES})"),
            Self::ImuStartup => f.write_str("IMU startup calibration failed"),
            Self::DuplicateDriveEncoders => {
                f.write_str("multiple differential motor groups supplied")
            }
        }
    }
}

impl std::error::Error for TrackingBuildError {}

mod sources;
use sources::Capture;
pub use sources::TrackingSources;

/// Live sources and their observations. Pose and estimator state belong to localization.
pub struct Tracking<S> {
    sources: S,
    #[cfg(feature = "vex")]
    differential: Option<usize>,
    #[cfg(feature = "vex")]
    imu: Option<usize>,
    range_layout: arrayvec::ArrayVec<(usize, RangeSensorConfig), MAX_RANGES>,
    wheel_mounts: Vec<(usize, DVec3)>,
    observations: Vec<Observation>,
}

impl<S: TrackingSources> TrackingSystem for Tracking<S> {
    #[cfg(feature = "vex")]
    fn drive_wheels(&self) -> Option<&DifferentialTracking> {
        self.sources.source(self.differential?)?.differential()
    }

    #[cfg(feature = "vex")]
    fn drive_wheels_mut(&mut self) -> Option<&mut DifferentialTracking> {
        self.sources
            .source_mut(self.differential?)?
            .differential_mut()
    }
}

impl<S: TrackingSources> Tracking<S> {
    /// Initialize the tuple or list of sources concurrently before the first sample.
    pub async fn new(mut sources: S) -> Result<Self, TrackingBuildError> {
        #[cfg(feature = "vex")]
        let mut differential = None;
        #[cfg(feature = "vex")]
        let mut imu = None;
        let mut range_layout = arrayvec::ArrayVec::new();
        let mut wheel_mounts = Vec::new();
        for index in 0..S::SOURCE_COUNT {
            let Some(source) = sources.source(index) else {
                continue;
            };
            #[cfg(feature = "vex")]
            {
                if source.differential().is_some() && differential.replace(index).is_some() {
                    return Err(TrackingBuildError::DuplicateDriveEncoders);
                }
                if imu.is_none() && source.inertial().is_some() {
                    imu = Some(index);
                }
            }
            if let Some(mount) = source.range_mount() {
                range_layout
                    .try_push((index, mount))
                    .map_err(|_| TrackingBuildError::RangeLayout)?;
            }
            if let Some(projection) = source.wheel_projection() {
                if !projection.is_finite() {
                    return Err(TrackingBuildError::WheelGeometry);
                }
                wheel_mounts.push((index, projection));
            }
        }
        for result in join_all(sources.initialization()).await {
            result?;
        }
        Ok(Self {
            observations: Vec::with_capacity(S::SOURCE_COUNT),
            range_layout,
            sources,
            #[cfg(feature = "vex")]
            differential,
            #[cfg(feature = "vex")]
            imu,
            wheel_mounts,
        })
    }

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
    /// Read orientation for driver assists and reuse it on the next sampling tick.
    pub fn refresh_orientation(&mut self) -> Option<Orientation> {
        self.sources
            .source_mut(self.imu?)?
            .inertial_mut()?
            .refresh(true)
    }

    #[cfg(feature = "vex")]
    pub fn refresh_heading(&mut self) -> Option<Orientation> {
        self.sources
            .source_mut(self.imu?)?
            .inertial_mut()?
            .refresh(false)
    }

    #[cfg(feature = "vex")]
    fn imu(&self) -> Option<&InertialSensorTracking> {
        self.sources.source(self.imu?)?.inertial()
    }

    #[cfg(feature = "vex")]
    pub fn diagnostic_orientation(&self) -> Option<Orientation> {
        self.imu()?.diagnostic_orientation()
    }

    #[cfg(feature = "vex")]
    pub fn imu_telemetry(&self) -> ([f64; 3], [f64; 3]) {
        self.imu().map_or(
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
    use ozton_math::{angle, length};

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
                    distance: length!(1 m),
                    confidence: 1.0,
                    occluded: sink.range_occluded,
                }));
            } else {
                sink.push(Measurement::Heading(HeadingObservation::Counterclockwise(
                    angle!(15 deg),
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
