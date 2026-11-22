//! Assemble a differential localizer from independently owned sensors.

use std::time::Duration;

use ozton_math::uom::si::f64::Length;
use ozton_tracking::localizer::{
    Estimator, Localizer, MAX_RANGES, RangeSensorConfig,
    vex::{DistanceTracker, Distances, Sensors},
    wheels::{MountedWheel, WheelOdometry},
};
use vexide::{prelude::InertialSensor, smart::SmartPort};

use super::DifferentialLocalization;

/// Ways an IMU can participate in tracking and control.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ImuUse {
    /// Use measured heading change in odometry.
    HeadingCorrection,
    /// Expose cached heading, pitch, roll and yaw to driver control.
    Orientation,
    /// Expose gyroscope and accelerometer readings through IMU telemetry.
    Telemetry,
}

/// Consumers of a distance sensor. A sensor can supply multiple consumers
/// without taking additional readings.
#[derive(Clone, Copy)]
pub enum DsrUse {
    /// On-demand geometric distance reset.
    Dsr,
    /// Range updates when EKF is selected.
    Ekf,
    /// Range observations when MCL is selected.
    Mcl,
}

#[derive(Default)]
pub(super) struct RangeUses {
    reset: u8,
    ekf: u8,
    mcl: u8,
}

impl RangeUses {
    fn add(&mut self, usage: DsrUse, index: usize) {
        let mask = match usage {
            DsrUse::Dsr => &mut self.reset,
            DsrUse::Ekf => &mut self.ekf,
            DsrUse::Mcl => &mut self.mcl,
        };
        *mask |= 1 << index;
    }

    pub(super) fn apply(&self, localizer: &mut Localizer<'_>) {
        let filter = match localizer.estimator() {
            Estimator::Odometry => 0,
            Estimator::Ekf(_) => self.ekf,
            Estimator::Mcl(_) => self.mcl,
        };
        localizer.set_range_usage(self.reset, filter);
    }
}

/// Starts empty and assembles one tracker for the selected sensor layout.
/// The drivetrain stays separately owned. Pass it to
/// [`DifferentialLocalization::poll_with_drive`] during a tracking tick.
#[derive(Default)]
pub struct TrackingBuilder {
    imu: Option<SmartPort>,
    heading: bool,
    orientation: bool,
    telemetry: bool,
    wheel_sources: Vec<MountedWheel>,
    drive_track_width: Option<Length>,
    ranges: Vec<DistanceTracker>,
    range_uses: RangeUses,
}

impl TrackingBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Configure the single IMU port and its enabled uses.
    pub fn with_imu(mut self, port: SmartPort, uses: impl IntoIterator<Item = ImuUse>) -> Self {
        self.imu = Some(port);
        self.heading = false;
        self.orientation = false;
        self.telemetry = false;
        for usage in uses {
            match usage {
                ImuUse::HeadingCorrection => self.heading = true,
                ImuUse::Orientation => self.orientation = true,
                ImuUse::Telemetry => self.telemetry = true,
            }
        }
        self
    }

    /// Append a group of mounted distance sensors in route index order.
    pub fn with_dsr(
        mut self,
        sensors: impl IntoIterator<Item = DistanceTracker>,
        uses: impl IntoIterator<Item = DsrUse>,
    ) -> Self {
        let first = self.ranges.len();
        self.ranges.extend(sensors);
        let end = self.ranges.len().min(MAX_RANGES);
        for usage in uses {
            for index in first..end {
                self.range_uses.add(usage, index);
            }
        }
        self
    }

    /// Append tracking wheels at arbitrary surveyed bearings.
    pub fn with_wheels(mut self, wheels: impl IntoIterator<Item = MountedWheel>) -> Self {
        self.wheel_sources.extend(wheels);
        self
    }

    /// Use the drivetrain's calibrated left and right encoders as forward
    /// wheels at half the track width on either side of the tracked origin.
    pub fn with_drive_encoders(mut self, track_width: Length) -> Self {
        self.drive_track_width = Some(track_width);
        self
    }

    /// Build the field model from mounts in registration order, then initialize
    /// the selected hardware. The localizer sets limits and selects the active
    /// estimator. Wheel positions and bearings determine odometry geometry.
    pub async fn build(
        self,
        field_model: impl FnOnce(&[RangeSensorConfig]) -> Option<Localizer<'static>>,
    ) -> Option<DifferentialLocalization<()>> {
        let distances = Distances::new(self.ranges)?;
        let localizer = field_model(distances.configs())?;
        if localizer.range_count() != distances.len() {
            return None;
        }
        let wheels = WheelOdometry::new(
            self.wheel_sources,
            self.drive_track_width,
            self.heading,
            localizer.odometry_config().require_lateral,
        )?;
        let distances = distances.initialize(Duration::from_secs(3)).await?;
        let mut sensors = Sensors::with_wheel_sources((), ())?.into_shared_reader();
        if let Some(port) = self
            .imu
            .filter(|_| self.heading || self.orientation || self.telemetry)
        {
            sensors = sensors.with_imu(InertialSensor::new(port)).await?;
        }
        Some(
            DifferentialLocalization::new(sensors, distances, localizer).with_layout(
                wheels,
                self.range_uses,
                self.orientation,
                self.telemetry,
            ),
        )
    }
}
