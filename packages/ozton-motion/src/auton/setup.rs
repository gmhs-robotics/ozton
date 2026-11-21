//! Build the stock differential chassis from one set of physical limits.

use std::{rc::Rc, time::Duration};

use ozton_control::drive::AntiTip;
use ozton_drivetrain::{
    Drivetrain,
    model::{Differential, TipAngleAxis},
};
use ozton_math::{
    geometry::Pose,
    localization::range::Surface,
    path::Limits,
    uom::{
        ConstZero,
        si::{
            angular_velocity::radian_per_second,
            f64::{Acceleration, Angle, AngularVelocity, Length, Velocity},
            length::meter,
        },
    },
};
use ozton_tracking::localizer::{
    Config, Estimator, Localizer, MAX_RANGES, OdometryConfig, RangeSensorConfig, Tracking,
    vex::{Distances, Sensors},
};
use vexide::{
    prelude::{DistanceSensor, InertialSensor, Motor},
    smart::imu::CalibrateError,
};

use super::{
    DriveOptions, Feedback,
    chassis::{Chassis, ChassisError, WheelControl},
};

/// Physical settings shared by the drivetrain, odometry, path preparation and runner.
#[derive(Clone)]
pub struct DifferentialConfig {
    pub wheel_diameter: Length,
    pub track_width: Length,
    pub free_wheel_speed: AngularVelocity,
    pub maximum_wheel_speed: Velocity,
    pub acceleration: Acceleration,
    pub deceleration: Acceleration,
    pub lateral_acceleration: Acceleration,
    pub max_control_gap: Duration,
    pub route_timeout: Duration,
    pub journal_capacity: usize,
    pub wheel_control: WheelControl,
    pub anti_tip: Option<(AntiTip, TipAngleAxis, Angle)>,
}

impl DifferentialConfig {
    pub const fn new(
        wheel_diameter: Length,
        track_width: Length,
        maximum_wheel_speed: Velocity,
        free_wheel_speed: AngularVelocity,
    ) -> Self {
        Self {
            wheel_diameter,
            track_width,
            free_wheel_speed,
            maximum_wheel_speed,
            acceleration: Acceleration {
                value: 1.0,
                ..Acceleration::ZERO
            },
            deceleration: Acceleration {
                value: 1.0,
                ..Acceleration::ZERO
            },
            lateral_acceleration: Acceleration {
                value: 1.0,
                ..Acceleration::ZERO
            },
            max_control_gap: Duration::from_millis(100),
            route_timeout: Duration::from_secs(15),
            journal_capacity: 0,
            wheel_control: WheelControl::Native,
            anti_tip: None,
        }
    }

    pub fn path_limits(&self) -> Limits {
        Limits::from_units(
            self.track_width,
            self.maximum_wheel_speed,
            self.maximum_wheel_speed,
            self.acceleration,
            self.deceleration,
            self.lateral_acceleration,
        )
    }

    pub async fn build(
        mut self,
        left: impl IntoIterator<Item = Motor>,
        right: impl IntoIterator<Item = Motor>,
        tracking: TrackingSetup,
    ) -> Result<ConfiguredChassis, SetupError> {
        if !self.path_limits().is_valid()
            || !self.wheel_diameter.value.is_finite()
            || self.wheel_diameter.value <= 0.0
            || self.max_control_gap < ozton_runtime::clock::CONTROL_PERIOD
            || self.route_timeout < ozton_runtime::clock::CONTROL_PERIOD
        {
            return Err(SetupError::DriveGeometry);
        }
        let left = build_motor_side(left)?;
        let right = build_motor_side(right)?;
        let mut model = Differential::new(left, right)
            .with_wheel_feedback(self.wheel_diameter, self.free_wheel_speed)
            .and_then(|model| model.with_native_speed_limit(self.maximum_wheel_speed))
            .ok_or(SetupError::DriveGeometry)?;

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
            max_correction,
        } = tracking;
        if ranges.len() > MAX_RANGES {
            return Err(SetupError::Tracking);
        }
        let (ports, mounts): (Vec<_>, Vec<_>) = ranges.into_iter().unzip();
        let distances = Distances::new(ports).ok_or(SetupError::Tracking)?;
        let localizer = Localizer::new_owned(
            Config {
                odometry: OdometryConfig {
                    track_width: self.track_width,
                    forward_offset,
                    lateral_offset,
                    require_lateral,
                    max_linear_speed: self.maximum_wheel_speed * 2.0,
                    max_angular_speed: AngularVelocity::new::<radian_per_second>(
                        self.maximum_wheel_speed.value * 3.0 / self.track_width.value,
                    ),
                    max_gap: self.max_control_gap,
                },
                estimator,
                shadow,
                max_correction,
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
        let control = std::mem::replace(&mut self.wheel_control, WheelControl::Native);
        let chassis = Chassis::new(
            Drivetrain::new(model, Tracking::new(localizer).with_distances(distances)),
            sensors.into_shared_reader(),
            control,
            self.journal_capacity,
        )
        .ok_or(SetupError::DriveGeometry)?;
        Ok(ConfiguredChassis {
            chassis,
            drive_options: DriveOptions::differential(self.track_width, self.maximum_wheel_speed),
            settings: self,
        })
    }
}

#[derive(Debug)]
pub enum SetupError {
    DriveGeometry,
    EmptyMotorSide,
    TooManyMotors,
    Tracking,
    MissingTipImu,
    Imu(CalibrateError),
}

/// VEX sensors, stationary reflecting surfaces and localization choices.
pub struct TrackingSetup {
    pub sensors: Sensors,
    pub estimator: Estimator,
    pub shadow: bool,
    /// Forward tracking wheel X-right position from The Origin, metres.
    pub forward_offset: Length,
    /// Right-positive lateral wheel Y-forward position from The Origin, metres.
    pub lateral_offset: Length,
    pub require_lateral: bool,
    pub max_correction: Length,
    pub imu: Option<InertialSensor>,
    pub map: Vec<Surface>,
    /// Sensor/config pairs in the order used by route indices.
    pub ranges: Vec<(DistanceSensor, RangeSensorConfig)>,
}

impl Default for TrackingSetup {
    fn default() -> Self {
        Self {
            sensors: Sensors::default(),
            estimator: Estimator::Odometry,
            shadow: false,
            forward_offset: Length::new::<meter>(0.0),
            lateral_offset: Length::new::<meter>(0.0),
            require_lateral: false,
            max_correction: Length::new::<meter>(0.16),
            imu: None,
            map: Vec::new(),
            ranges: Vec::new(),
        }
    }
}

/// A chassis with the physical limits and sensors used by the stock robot adapter.
pub struct ConfiguredChassis {
    pub(super) chassis: Chassis<'static>,
    pub(super) settings: DifferentialConfig,
    pub(super) drive_options: DriveOptions,
}

impl ConfiguredChassis {
    pub fn chassis(&self) -> &Chassis<'static> {
        &self.chassis
    }

    pub fn sample(&mut self, occluded: u8) -> Result<Option<Feedback>, ChassisError> {
        self.chassis.sample(occluded)
    }

    pub fn drive(&mut self, target: [Velocity; 2], dt: Duration) -> Result<(), ChassisError> {
        self.chassis.drive(target, dt)
    }

    pub fn stop(&mut self) -> Result<(), ChassisError> {
        self.chassis.stop()
    }

    pub fn drive_driver_arcade(
        &mut self,
        throttle: f64,
        steer: f64,
        policy: ozton_drivetrain::model::DriverFaultPolicy,
    ) -> Result<(), vexide::smart::PortError> {
        self.chassis.drive_driver_arcade(throttle, steer, policy)
    }

    pub fn journal_mut(&mut self) -> &mut ozton_diagnostics::Journal {
        &mut self.chassis.journal
    }

    pub fn set_pose(&mut self, pose: Pose) -> Result<(), ChassisError> {
        self.chassis.set_pose(pose)
    }

    pub fn request_distance_reset(&mut self) {
        self.chassis.request_distance_reset();
    }

    pub fn cancel_distance_reset(&mut self) {
        self.chassis.cancel_distance_reset();
    }

    pub fn log_step(
        &mut self,
        elapsed: Duration,
        dt: Duration,
        progress: Length,
        wheels: [Velocity; 2],
    ) {
        self.chassis.log_step(elapsed, dt, progress, wheels);
    }

    pub fn range_count(&self) -> usize {
        self.chassis.drivetrain.tracking.localizer.range_count()
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

fn build_motor_side(motors: impl IntoIterator<Item = Motor>) -> Result<Vec<Motor>, SetupError> {
    let mut side = Vec::new();
    for motor in motors {
        if side.len() == ozton_drivetrain::feedback::MAX_SIDE_MOTORS {
            return Err(SetupError::TooManyMotors);
        }
        side.push(motor);
    }
    if side.is_empty() {
        return Err(SetupError::EmptyMotorSide);
    }
    Ok(side)
}
