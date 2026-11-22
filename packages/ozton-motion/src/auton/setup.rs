//! Build the stock differential chassis from one set of physical limits.

use std::time::Duration;

use ozton_drivetrain::{
    Drivetrain,
    model::{ArcadeInput, Differential, DriverFaultPolicy, DrivetrainModel, ProgramDrive},
};
use ozton_math::{
    geometry::Pose,
    localization::range::Surface,
    path::Limits,
    uom::{
        ConstZero,
        si::{
            angular_velocity::radian_per_second,
            electric_potential::volt,
            f64::{Acceleration, AngularVelocity, ElectricPotential, Length, Velocity},
            length::meter,
        },
    },
};
use ozton_tracking::localizer::{
    Config, Estimator, Localizer, OdometryConfig, Tracking,
    vex::{DistanceTracker, Distances, Sensors},
};
use vexide::prelude::{InertialSensor, Motor};

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
        let model = Differential::new(left, right)
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
        let distances = Distances::new(ranges)
            .ok_or(SetupError::Tracking)?
            .initialize(Duration::from_secs(3))
            .await
            .ok_or(SetupError::Tracking)?;
        let mounts = distances.configs().to_vec();
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
        if let Some(imu) = imu {
            sensors = sensors.with_imu(imu).await.ok_or(SetupError::Tracking)?;
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
    /// Mounted sensors in route index order.
    pub ranges: Vec<DistanceTracker>,
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

/// Manual arcade input or bounded output from a supervised calibration.
pub enum ChassisDriverCommand {
    Arcade(ArcadeInput),
    Calibration(super::calibration::CalibrationOutput),
}

impl From<ArcadeInput> for ChassisDriverCommand {
    fn from(input: ArcadeInput) -> Self {
        Self::Arcade(input)
    }
}

impl DrivetrainModel for ConfiguredChassis {
    type Error = ChassisError;
}

impl ProgramDrive for ConfiguredChassis {
    type DriverCommand = ChassisDriverCommand;
    type AutonCommand = [Velocity; 2];

    fn apply_driver(
        &mut self,
        command: Self::DriverCommand,
        _: Duration,
    ) -> Result<(), Self::Error> {
        match command {
            ChassisDriverCommand::Arcade(input) => self
                .drive_driver_arcade(input.throttle, input.turn, DriverFaultPolicy::Stop)
                .map_err(ChassisError::Output),
            ChassisDriverCommand::Calibration(output) => {
                let [left, right] = output.volts().map(ElectricPotential::new::<volt>);
                self.chassis
                    .drivetrain
                    .model
                    .drive_volts(left, right)
                    .map_err(ChassisError::Output)
            }
        }
    }

    fn apply_auton(
        &mut self,
        command: Self::AutonCommand,
        dt: Duration,
    ) -> Result<(), Self::Error> {
        self.drive(command, dt)
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        ConfiguredChassis::stop(self)
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
