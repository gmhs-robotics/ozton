//! Build the stock differential chassis from one set of physical limits.

use std::time::Duration;

use ozton_drivetrain::{
    Drivetrain,
    model::{Differential, DriveConfigError},
};
use ozton_localization::localizer::{Config, Estimator, Localizer, OdometryConfig};
use ozton_math::{
    geometry::Pose,
    localization::range::Surface,
    path::Limits,
    uom::{
        ConstZero,
        si::{
            electric_potential::volt,
            f64::{Acceleration, AngularVelocity, ElectricPotential, Length, Velocity},
        },
    },
};
use ozton_motion::arcade::{Arcade, ArcadeInput};
use ozton_runtime::competition::ProgramDrive;
use ozton_tracking::{
    odometric::DifferentialTracking,
    range::{DistanceTracker, Distances},
    sampling::Sensors,
};
use vexide::prelude::{InertialSensor, Motor};

use super::{
    DriveOptions, Feedback,
    chassis::{Chassis, ChassisError, ChassisSensors, WheelControl},
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
        {
            return Err(SetupError::DriveGeometry);
        }
        let left = build_motor_side(left)?;
        let right = build_motor_side(right)?;
        let TrackingSetup {
            sensors,
            imu,
            map,
            ranges,
            estimator,
            shadow,
            forward_offset,
            lateral_offset,
        } = tracking;
        let distances = Distances::new(ranges).ok_or(SetupError::Tracking)?;
        let mounts = distances.configs().to_vec();
        let localizer = Localizer::new_owned(
            Config {
                odometry: OdometryConfig {
                    track_width: self.track_width,
                    forward_offset,
                    lateral_offset,
                    max_gap: self.max_control_gap,
                },
                estimator,
                shadow,
            },
            Pose::default(),
            map,
            mounts,
        )
        .ok_or(SetupError::Tracking)?;
        let (sensors, distances) = sensors
            .into_shared_reader()
            .initialize_with_distances(imu, distances)
            .await
            .ok_or(SetupError::Tracking)?;
        let control = std::mem::replace(&mut self.wheel_control, WheelControl::Native);
        let model = Differential::new(left, right);
        let drive_encoders = DifferentialTracking::inferred(
            model.shared_motors(),
            self.track_width,
            self.wheel_diameter,
            self.free_wheel_speed,
        )?;
        let drivetrain = Drivetrain::new(
            model,
            ChassisSensors {
                sensors,
                distances,
                drive_encoders,
            },
            localizer,
        );
        let chassis = Chassis::new(drivetrain, control, self.journal_capacity)
            .ok_or(SetupError::DriveGeometry)?;
        Ok(ConfiguredChassis {
            chassis,
            drive_options: DriveOptions::differential(self.track_width, self.maximum_wheel_speed),
            settings: self,
            motion_config: None,
        })
    }
}

#[derive(Debug)]
pub enum SetupError {
    DriveGeometry,
    DriveConfig(DriveConfigError),
    EmptyMotorSide,
    TooManyMotors,
    Tracking,
}

impl From<DriveConfigError> for SetupError {
    fn from(error: DriveConfigError) -> Self {
        Self::DriveConfig(error)
    }
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
            forward_offset: ozton_math::length!(0.0 m),
            lateral_offset: ozton_math::length!(0.0 m),
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
    motion_config: Option<ozton_motion::MotionConfig>,
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
        self.chassis.drivetrain.localization().range_count()
    }

    pub fn drive_options(&self) -> DriveOptions {
        self.drive_options
    }

    pub fn motion_config(&self) -> Option<ozton_motion::MotionConfig> {
        self.motion_config
    }

    /// Install the application's calibrated motion policy on the stock chassis.
    pub fn set_motion_config(
        &mut self,
        config: ozton_motion::MotionConfig,
    ) -> Result<(), SetupError> {
        if !config.valid() || config.wheel_speed > self.settings.maximum_wheel_speed {
            return Err(SetupError::DriveGeometry);
        }
        self.drive_options = config.options(self.settings.track_width);
        self.motion_config = Some(config);
        Ok(())
    }

    /// Change route limits and settling behavior while retaining the calibrated track width.
    pub fn set_drive_options(&mut self, mut options: DriveOptions) -> Result<(), SetupError> {
        options.track_width = self.settings.track_width;
        if !options.valid() {
            return Err(SetupError::DriveGeometry);
        }
        self.drive_options = options;
        if let Some(config) = &mut self.motion_config {
            config.wheel_speed = options.maximum_wheel_speed;
            config.position_tolerance = options.position_tolerance;
            config.heading_tolerance = options.heading_tolerance;
            config.velocity_tolerance = options.speed_tolerance;
            config.angular_velocity_tolerance = options.angular_speed_tolerance;
            config.settle = options.settle;
        }
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

impl ozton_runtime::diagnostics::ComponentReport for ConfiguredChassis {
    fn report(&self, frame: ozton_runtime::diagnostics::Frame) {
        ozton_runtime::diagnostics::report_pose(
            frame,
            self.chassis.drivetrain.localization().pose(),
            self.chassis.pose_trusted(),
        );
    }
}

impl ProgramDrive for ConfiguredChassis {
    type Error = ChassisError;
    type DriverCommand = ChassisDriverCommand;
    type AutonCommand = [Velocity; 2];

    fn apply_driver(
        &mut self,
        command: Self::DriverCommand,
        _: Duration,
    ) -> Result<(), Self::Error> {
        match command {
            ChassisDriverCommand::Arcade(input) => {
                let (throttle, turn) = input.scaled_axes();
                Arcade::drive(&mut self.chassis.drivetrain, throttle, turn)
                    .map_err(ChassisError::Output)
            }
            ChassisDriverCommand::Calibration(output) => {
                let volts = output.volts().map(ElectricPotential::new::<volt>);
                self.chassis
                    .drivetrain
                    .drive_volts(volts)
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
        if side.len() == ozton_tracking::odometric::consensus::MAX_SIDE_MOTORS {
            return Err(SetupError::TooManyMotors);
        }
        side.push(motor);
    }
    if side.is_empty() {
        return Err(SetupError::EmptyMotorSide);
    }
    Ok(side)
}
