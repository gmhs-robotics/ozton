//! Assemble a differential localizer from independently owned sensors.

use std::time::Duration;

use ozton_drivetrain::model::DriveEncoders;
use ozton_tracking::localizer::{
    Estimator, Localizer, MAX_RANGES, RangeSensorConfig,
    vex::{DistanceTracker, Distances, Sensors},
    wheels::{DriveWheels, MountedWheel, WheelOdometry},
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
}

/// Which motion components the differential drive encoders may estimate.
#[derive(Clone, Copy)]
pub enum DriveEncoderUse {
    /// Average left and right travel for forward motion.
    Forward,
    /// Also use the wheel travel difference for yaw when the IMU and pods
    /// cannot provide it. Uses the track width on the installed drivetrain.
    ForwardAndHeading,
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
    wheel_sources: Vec<MountedWheel>,
    drive: Option<DriveEncoderUse>,
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
        for usage in uses {
            match usage {
                ImuUse::HeadingCorrection => self.heading = true,
                ImuUse::Orientation => self.orientation = true,
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

    /// Use calibrated drive-wheel travel for the selected motion components.
    pub fn with_drive_encoders(mut self, usage: DriveEncoderUse) -> Self {
        self.drive = Some(usage);
        self
    }

    /// Build tracking with pods alone or with forward-only drive travel.
    /// For drive-derived yaw, use [`Self::build_for_drive`].
    pub async fn build(
        self,
        field_model: impl FnOnce(&[RangeSensorConfig]) -> Option<Localizer<'static>>,
    ) -> Option<DifferentialLocalization<()>> {
        let drive = match self.drive {
            Some(DriveEncoderUse::ForwardAndHeading) => return None,
            Some(DriveEncoderUse::Forward) => Some(DriveWheels::Forward),
            None => None,
        };
        self.build_layout(drive, field_model).await
    }

    /// Resolve encoder geometry from the robot's installed drivetrain at assembly.
    pub async fn build_for_drive(
        self,
        drivetrain: &impl DriveEncoders,
        field_model: impl FnOnce(&[RangeSensorConfig]) -> Option<Localizer<'static>>,
    ) -> Option<DifferentialLocalization<()>> {
        let drive = match self.drive {
            Some(DriveEncoderUse::ForwardAndHeading) => {
                Some(DriveWheels::ForwardAndHeading(drivetrain.track_width()?))
            }
            Some(DriveEncoderUse::Forward) => Some(DriveWheels::Forward),
            None => None,
        };
        self.build_layout(drive, field_model).await
    }

    async fn build_layout(
        self,
        drive: Option<DriveWheels>,
        field_model: impl FnOnce(&[RangeSensorConfig]) -> Option<Localizer<'static>>,
    ) -> Option<DifferentialLocalization<()>> {
        let distances = Distances::new(self.ranges)?;
        let localizer = field_model(distances.configs())?;
        if localizer.range_count() != distances.len() {
            return None;
        }
        let wheels = WheelOdometry::new(
            self.wheel_sources,
            drive,
            self.heading,
            localizer.odometry_config().require_lateral,
        )?;
        let distances = distances.initialize(Duration::from_secs(3)).await?;
        let mut sensors = Sensors::with_wheel_sources((), ())?.into_shared_reader();
        if let Some(port) = self.imu {
            sensors = sensors.with_imu(InertialSensor::new(port)).await?;
        }
        Some(
            DifferentialLocalization::new(sensors, distances, localizer).with_layout(
                wheels,
                self.range_uses,
                self.orientation,
            ),
        )
    }
}
