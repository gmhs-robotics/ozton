//! Build the stock differential chassis from one set of physical limits.

use std::{rc::Rc, time::Duration};

use ozton_control::drive::AntiTip;
use ozton_drivetrain::{
    Drivetrain,
    model::{Differential, DriveMotor, GearRatio, TipAngleAxis},
};
use ozton_math::{geometry::Pose, localization::range::Surface, path::Limits};
use ozton_tracking::localizer::{
    Config, Estimator, Localizer, MAX_RANGES, OdometryConfig, RangeSensorConfig, vex::Sensors,
};
use vexide::{
    math::Angle,
    prelude::{DistanceSensor, InertialSensor, Motor},
    smart::imu::CalibrateError,
};

use super::{
    DriveOptions,
    chassis::{Chassis, WheelControl},
};

/// Physical settings shared by the drivetrain, odometry, path preparation and runner.
#[derive(Clone, Copy)]
pub struct DifferentialConfig {
    pub wheel_diameter: f64,
    pub track_width: f64,
    pub maximum_wheel_speed: f64,
    pub acceleration: f64,
    pub deceleration: f64,
    pub lateral_acceleration: f64,
    pub max_control_gap: Duration,
    pub route_timeout: Duration,
    pub journal_capacity: usize,
    pub wheel_control: WheelControl,
    pub anti_tip: Option<(AntiTip, TipAngleAxis, Angle)>,
}

impl DifferentialConfig {
    pub const fn new(wheel_diameter: f64, track_width: f64, maximum_wheel_speed: f64) -> Self {
        Self {
            wheel_diameter,
            track_width,
            maximum_wheel_speed,
            acceleration: 1.0,
            deceleration: 1.0,
            lateral_acceleration: 1.0,
            max_control_gap: Duration::from_millis(100),
            route_timeout: Duration::from_secs(15),
            journal_capacity: 0,
            wheel_control: WheelControl::Native,
            anti_tip: None,
        }
    }

    pub fn path_limits(self) -> Limits {
        Limits {
            track_width: self.track_width,
            linear_speed: self.maximum_wheel_speed,
            wheel_speed: self.maximum_wheel_speed,
            acceleration: self.acceleration,
            deceleration: self.deceleration,
            lateral_acceleration: self.lateral_acceleration,
        }
    }

    pub async fn build(
        self,
        left: impl IntoIterator<Item = (Motor, GearRatio)>,
        right: impl IntoIterator<Item = (Motor, GearRatio)>,
        tracking: TrackingSetup,
    ) -> Result<ConfiguredChassis, SetupError> {
        if !self.path_limits().is_valid()
            || !self.wheel_diameter.is_finite()
            || self.wheel_diameter <= 0.0
            || self.max_control_gap < ozton_runtime::clock::CONTROL_PERIOD
            || self.route_timeout < ozton_runtime::clock::CONTROL_PERIOD
        {
            return Err(SetupError::DriveGeometry);
        }
        let build_side = |(motor, ratio)| {
            DriveMotor::new(motor, ratio, self.wheel_diameter).ok_or(SetupError::DriveGeometry)
        };
        let left = left
            .into_iter()
            .map(build_side)
            .collect::<Result<Vec<_>, _>>()?;
        let right = right
            .into_iter()
            .map(build_side)
            .collect::<Result<Vec<_>, _>>()?;
        let mut model = Differential::geared(left, right, Some(self.maximum_wheel_speed))
            .ok_or(SetupError::EmptyMotorSide)?;

        let TrackingSetup {
            mut sensors,
            imu,
            map,
            ranges,
            estimator,
            shadow,
            forward_offset,
            lateral_offset,
            require_lateral,
            max_correction_meters,
        } = tracking;
        let mut names = Vec::with_capacity(ranges.len());
        let mut ports = Vec::with_capacity(ranges.len());
        let mut mounts = Vec::with_capacity(ranges.len());
        for (name, port, mount) in ranges {
            if name.trim().is_empty() || names.contains(&name) {
                return Err(SetupError::RangeName(name));
            }
            names.push(name);
            ports.push(port);
            mounts.push(mount);
        }
        if names.len() > MAX_RANGES {
            return Err(SetupError::Tracking);
        }
        sensors = sensors.with_ranges(ports).ok_or(SetupError::Tracking)?;
        let localizer = Localizer::new_owned(
            Config {
                odometry: OdometryConfig {
                    track_width: self.track_width,
                    forward_offset,
                    lateral_offset,
                    require_lateral,
                    max_linear_speed: self.maximum_wheel_speed * 2.0,
                    max_angular_speed: self.maximum_wheel_speed * 3.0 / self.track_width,
                    max_gap: self.max_control_gap,
                },
                estimator,
                shadow,
                max_correction_meters,
            },
            Pose::default(),
            map,
            mounts,
        )
        .ok_or(SetupError::Tracking)?;
        if let Some(mut imu) = imu {
            imu.calibrate().await.map_err(SetupError::Imu)?;
            sensors.imu = Some(Rc::new(imu));
        }
        if let Some((controller, axis, level)) = self.anti_tip {
            let imu = sensors.imu.clone().ok_or(SetupError::MissingTipImu)?;
            model = model.with_anti_tip(imu, controller, axis, level);
        }
        let chassis = Chassis::new(
            Drivetrain::new(model, localizer),
            sensors,
            self.wheel_control,
            self.journal_capacity,
        )
        .ok_or(SetupError::DriveGeometry)?;
        Ok(ConfiguredChassis {
            chassis,
            settings: self,
            names,
            drive_options: DriveOptions::differential(self.track_width, self.maximum_wheel_speed),
        })
    }
}

#[derive(Debug)]
pub enum SetupError {
    DriveGeometry,
    EmptyMotorSide,
    Tracking,
    RangeName(&'static str),
    MissingTipImu,
    Imu(CalibrateError),
}

/// VEX sensors, stationary reflecting surfaces and localization choices.
pub struct TrackingSetup {
    pub sensors: Sensors,
    pub estimator: Estimator,
    pub shadow: bool,
    pub forward_offset: f64,
    pub lateral_offset: f64,
    pub require_lateral: bool,
    pub max_correction_meters: f64,
    imu: Option<InertialSensor>,
    map: Vec<Surface>,
    ranges: Vec<(&'static str, DistanceSensor, RangeSensorConfig)>,
}

impl Default for TrackingSetup {
    fn default() -> Self {
        Self {
            sensors: Sensors::default(),
            estimator: Estimator::Odometry,
            shadow: false,
            forward_offset: 0.0,
            lateral_offset: 0.0,
            require_lateral: false,
            max_correction_meters: 0.16,
            imu: None,
            map: Vec::new(),
            ranges: Vec::new(),
        }
    }
}

impl TrackingSetup {
    /// The builder calibrates this IMU before returning the chassis.
    pub fn with_imu(mut self, imu: InertialSensor) -> Self {
        self.imu = Some(imu);
        self
    }

    pub fn with_map(mut self, surfaces: impl IntoIterator<Item = Surface>) -> Self {
        self.map = surfaces.into_iter().collect();
        self
    }

    /// Register a named device and its mount together.
    pub fn with_range(
        mut self,
        name: &'static str,
        sensor: DistanceSensor,
        config: RangeSensorConfig,
    ) -> Self {
        self.ranges.push((name, sensor, config));
        self
    }
}

/// Opaque identifier for a registered range sensor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RangeId(pub(super) usize);

impl RangeId {
    pub const fn mask(self) -> u8 {
        1 << self.0
    }
}

/// A chassis with the limits and sensor names required by the stock robot adapter.
pub struct ConfiguredChassis {
    pub chassis: Chassis<'static>,
    pub(super) settings: DifferentialConfig,
    pub(super) drive_options: DriveOptions,
    names: Vec<&'static str>,
}

impl ConfiguredChassis {
    pub fn range(&self, name: &str) -> Option<RangeId> {
        self.names
            .iter()
            .position(|candidate| *candidate == name)
            .map(RangeId)
    }

    pub fn drive_options(&self) -> DriveOptions {
        self.drive_options
    }

    /// Override settling behavior while keeping the physical limits synchronized.
    pub fn set_drive_options(&mut self, mut options: DriveOptions) -> Result<(), SetupError> {
        options.track_width = self.settings.track_width;
        options.maximum_wheel_speed = self.settings.maximum_wheel_speed;
        if !options.valid() {
            return Err(SetupError::DriveGeometry);
        }
        self.drive_options = options;
        Ok(())
    }
}
