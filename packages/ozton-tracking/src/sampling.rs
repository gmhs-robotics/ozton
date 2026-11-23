//! Sampling hardware independently of pose estimation.
use glam::DVec3;
use ozton_math::uom::si::f64::Length;
use vexide::{prelude::InertialSensor, smart::SmartPort};

use crate::{
    Tracking,
    sources::{
        Snapshot,
        distance::{MAX_RANGES, RangeObservation, RangeSensorConfig},
        vex::{DistanceTracker, Distances, Orientation, Sensors, SharedSensors},
        wheels::MountedWheel,
    },
};

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
            Self::RangeLayout => write!(f, "too many distance sensors (maximum {MAX_RANGES})"),
            Self::ImuStartup => f.write_str("IMU startup calibration failed"),
        }
    }
}

impl std::error::Error for TrackingBuildError {}

/// Install sensors and their physical mounts. Building initializes the devices once.
#[derive(Default)]
pub struct TrackingBuilder {
    imu: Option<SmartPort>,
    drive_encoders: bool,
    wheels: Vec<MountedWheel>,
    ranges: Vec<DistanceTracker>,
}

impl TrackingBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_imu(mut self, port: SmartPort) -> Self {
        self.imu = Some(port);
        self
    }

    /// Sample the calibrated encoders on the model that this tracker is attached to.
    pub fn with_drive_encoders(mut self) -> Self {
        self.drive_encoders = true;
        self
    }

    pub fn with_wheels(mut self, wheels: impl IntoIterator<Item = MountedWheel>) -> Self {
        self.wheels.extend(wheels);
        self
    }

    /// Register mounted distance sensors in observation order.
    pub fn with_dsr(mut self, sensors: impl IntoIterator<Item = DistanceTracker>) -> Self {
        self.ranges.extend(sensors);
        self
    }

    pub async fn build(self) -> Result<SensorTracking, TrackingBuildError> {
        if self.wheels.iter().any(|wheel| !wheel.is_valid()) {
            return Err(TrackingBuildError::WheelGeometry);
        }
        let distances = Distances::new(self.ranges)
            .ok_or(TrackingBuildError::RangeLayout)?
            .initialize()
            .await;
        let mut sensors = Sensors::default().into_shared_reader();
        if let Some(port) = self.imu {
            sensors = sensors
                .with_imu(InertialSensor::new(port))
                .await
                .ok_or(TrackingBuildError::ImuStartup)?;
        }
        Ok(SensorTracking {
            sensors,
            distances,
            drive_encoders: self.drive_encoders,
            positions: vec![None; self.wheels.len()],
            wheels: self.wheels,
            orientation_sampled: false,
        })
    }
}

/// Live hardware sources. This type never stores or updates a pose.
pub struct SensorTracking {
    sensors: SharedSensors,
    distances: Distances,
    wheels: Vec<MountedWheel>,
    positions: Vec<Option<Length>>,
    drive_encoders: bool,
    orientation_sampled: bool,
}

impl Tracking for SensorTracking {}

pub struct SensorSample<'a> {
    pub snapshot: Snapshot,
    pub wheels: &'a [Option<Length>],
    pub ranges: [Option<RangeObservation>; MAX_RANGES],
}

impl SensorTracking {
    pub fn uses_drive_encoders(&self) -> bool {
        self.drive_encoders
    }

    pub fn wheel_projections(&self) -> impl Iterator<Item = DVec3> + '_ {
        self.wheels.iter().map(MountedWheel::projection)
    }

    pub fn range_mounts(&self) -> &[RangeSensorConfig] {
        self.distances.configs()
    }

    /// Refresh orientation for driver assists, and reuse it for the next sample.
    pub fn refresh_orientation(&mut self) -> Option<Orientation> {
        self.orientation_sampled = true;
        self.sensors.refresh_orientation()
    }

    pub fn refresh_heading(&mut self) -> Option<Orientation> {
        self.orientation_sampled = true;
        self.sensors.refresh_heading()
    }

    pub fn diagnostic_orientation(&self) -> Option<Orientation> {
        self.sensors.diagnostic_orientation()
    }

    pub fn imu_telemetry(&self) -> ([f64; 3], [f64; 3]) {
        self.sensors.imu_telemetry()
    }

    pub fn capture(&mut self, drive: [Option<Length>; 2], occluded: u8) -> SensorSample<'_> {
        if !std::mem::take(&mut self.orientation_sampled) {
            self.sensors.refresh_heading();
        }
        for (reading, wheel) in self.positions.iter_mut().zip(&self.wheels) {
            *reading = wheel.sample();
        }
        let drive = if self.drive_encoders {
            drive
        } else {
            [None; 2]
        };
        let (snapshot, ranges) =
            self.sensors
                .capture_with_distances(&mut self.distances, drive, occluded);
        SensorSample {
            snapshot,
            wheels: &self.positions,
            ranges,
        }
    }
}

/// Localization lifecycle for a differential drivetrain in a programmed competition.
pub trait LocalizationLifecycle<T> {
    fn mode_changed(&mut self, _mode: vexide::competition::CompetitionMode) {}
    fn cancel_pending(&mut self) {}
    fn uses_drive_encoders(&self, _tracking: &T) -> bool {
        false
    }
    fn poll(&mut self, _tracking: &mut T, _positions: [Option<Length>; 2], _width: Option<Length>) {
    }
}

impl<T> LocalizationLifecycle<T> for () {}
