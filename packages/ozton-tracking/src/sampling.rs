//! Sample a tuple or list of tracking sources without estimating pose.

use std::time::Duration;

use futures_util::future::join_all;
use glam::DVec3;
use vexide::time::LowResolutionTime;

use crate::{
    MeasurementSink, Observation, TrackingSource, TrackingSystem,
    inertial::{InertialSensorTracking, Orientation},
    odometric::DifferentialTracking,
    range::{MAX_RANGES, RangeSensorConfig},
};

mod vex;
pub use vex::{Sensors, SharedSensors, WheelSource};

pub(crate) fn stamp(time: LowResolutionTime) -> Duration {
    time.duration_since(LowResolutionTime::EPOCH)
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

/// A heterogeneous tuple or a list of boxed sources, in registration order.
pub trait TrackingSources {
    fn into_sources(self) -> Vec<Box<dyn TrackingSource>>;
}

impl TrackingSources for Vec<Box<dyn TrackingSource>> {
    fn into_sources(self) -> Vec<Box<dyn TrackingSource>> {
        self
    }
}

impl TrackingSources for () {
    fn into_sources(self) -> Vec<Box<dyn TrackingSource>> {
        Vec::new()
    }
}

macro_rules! tracking_tuples {
    () => {};
    ($first:ident $(, $rest:ident)*) => {
        impl<$first: TrackingSource + 'static, $($rest: TrackingSource + 'static),*> TrackingSources
            for ($first, $($rest,)*)
        {
            #[allow(non_snake_case)]
            fn into_sources(self) -> Vec<Box<dyn TrackingSource>> {
                let ($first, $($rest,)*) = self;
                vec![Box::new($first), $(Box::new($rest)),*]
            }
        }

        tracking_tuples!($($rest),*);
    };
}

tracking_tuples!(A, B, C, D, E, F, G, H, I, J, K, L);

/// Live sources and their observations. Pose and estimator state belong to localization.
pub struct Tracking {
    sources: Vec<Box<dyn TrackingSource>>,
    differential: Option<usize>,
    imu: Option<usize>,
    range_mounts: Vec<RangeSensorConfig>,
    range_sources: Vec<usize>,
    wheel_mounts: Vec<(usize, DVec3)>,
    observations: Vec<Observation>,
}

/// Observations captured on the same sampling tick, in source order.
pub struct SensorSample<'a> {
    pub time: Duration,
    pub observations: &'a [Observation],
}

impl TrackingSystem for Tracking {
    fn drive_wheels(&self) -> Option<&DifferentialTracking> {
        self.sources.get(self.differential?)?.differential()
    }

    fn drive_wheels_mut(&mut self) -> Option<&mut DifferentialTracking> {
        self.sources.get_mut(self.differential?)?.differential_mut()
    }
}

impl Tracking {
    /// Initialize the tuple or list of sources concurrently before the first sample.
    pub async fn new(sources: impl TrackingSources) -> Result<Self, TrackingBuildError> {
        let mut sources = sources.into_sources();
        let mut differential = None;
        let mut imu = None;
        let mut range_mounts = Vec::new();
        let mut range_sources = Vec::new();
        let mut wheel_mounts = Vec::new();
        for (index, source) in sources.iter().enumerate() {
            if source.differential().is_some() && differential.replace(index).is_some() {
                return Err(TrackingBuildError::DuplicateDriveEncoders);
            }
            if imu.is_none() && source.inertial().is_some() {
                imu = Some(index);
            }
            if let Some(mount) = source.range_mount() {
                range_mounts.push(mount);
                range_sources.push(index);
            }
            if let Some(projection) = source.wheel_projection() {
                if !projection.is_finite() {
                    return Err(TrackingBuildError::WheelGeometry);
                }
                wheel_mounts.push((index, projection));
            }
        }
        if range_mounts.len() > MAX_RANGES {
            return Err(TrackingBuildError::RangeLayout);
        }
        for result in join_all(sources.iter_mut().filter_map(|source| source.initialize())).await {
            result?;
        }
        Ok(Self {
            observations: Vec::with_capacity(sources.len()),
            sources,
            differential,
            imu,
            range_mounts,
            range_sources,
            wheel_mounts,
        })
    }

    pub fn wheel_mounts(&self) -> &[(usize, DVec3)] {
        &self.wheel_mounts
    }

    pub fn range_sources(&self) -> &[usize] {
        &self.range_sources
    }

    pub fn range_mounts(&self) -> &[RangeSensorConfig] {
        &self.range_mounts
    }

    /// Read orientation for driver assists and reuse it on the next sampling tick.
    pub fn refresh_orientation(&mut self) -> Option<Orientation> {
        self.sources
            .get_mut(self.imu?)?
            .inertial_mut()?
            .refresh(true)
    }

    pub fn refresh_heading(&mut self) -> Option<Orientation> {
        self.sources
            .get_mut(self.imu?)?
            .inertial_mut()?
            .refresh(false)
    }

    fn imu(&self) -> Option<&InertialSensorTracking> {
        self.sources.get(self.imu?)?.inertial()
    }

    pub fn diagnostic_orientation(&self) -> Option<Orientation> {
        self.imu()?.diagnostic_orientation()
    }

    pub fn imu_telemetry(&self) -> ([f64; 3], [f64; 3]) {
        self.imu().map_or(
            ([f64::NAN; 3], [f64::NAN; 3]),
            InertialSensorTracking::telemetry,
        )
    }

    pub fn sample(&mut self, occluded_ranges: u8) -> SensorSample<'_> {
        let time = stamp(LowResolutionTime::now());
        self.observations.clear();
        let mut range_sources = self.range_sources.iter().copied().enumerate().peekable();
        for (index, source) in self.sources.iter_mut().enumerate() {
            let range_index = range_sources
                .next_if(|&(_, source)| source == index)
                .map(|(range, _)| range);
            let range_occluded =
                range_index.is_some_and(|range| occluded_ranges & (1 << range) != 0);
            source.sample(&mut MeasurementSink {
                time,
                source: index,
                range_occluded,
                range_index,
                observations: &mut self.observations,
            });
        }
        SensorSample {
            time,
            observations: &self.observations,
        }
    }
}

/// Localization lifecycle for a drivetrain in a programmed competition.
pub trait LocalizationLifecycle<T> {
    fn mode_changed(&mut self, _mode: vexide::competition::CompetitionMode) {}
    fn cancel_pending(&mut self) {}
    fn poll(&mut self, _tracking: &mut T) {}
}

impl<T> LocalizationLifecycle<T> for () {}

// WheeledTracking publishes its estimate from its own sampling task.
impl<T> LocalizationLifecycle<T> for crate::WheeledTracking {}
