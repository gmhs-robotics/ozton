//! A motion model, the robot's physical geometry, and its tracking system.
use std::time::{Duration, Instant};

use model::{
    ArcadeInput, Differential, DriveEncoders, DrivetrainModel, Holonomic, Mecanum, ProgramDrive,
    Tank,
};
use ozton_control::{drive::AntiTip, heading_hold::HeadingHold};
use ozton_math::uom::si::f64::{Angle, AngularVelocity, ElectricPotential, Length, Velocity};
use ozton_tracking::Tracking;
use vexide::{
    prelude::Motor,
    smart::{PortError, motor::BrakeMode},
};
pub use wheels::{DriveConfigError, WheelFeedback, WheelGearing, WheelLoad, WheelSpeedError};
use wheels::{Wheels, positive};

mod driver;
pub mod feedback;
pub mod model;
mod wheels;

/// Access the model's motor groups in their fixed wheel order.
pub trait MotorGroups {
    fn motor_groups(&self) -> &[Box<[Motor]>];
    fn motor_groups_mut(&mut self) -> &mut [Box<[Motor]>];
}

/// A drivetrain owns geometry and feedback calibration independently of how it mixes motor outputs.
pub struct Drivetrain<M: DrivetrainModel, T: Tracking, L> {
    pub model: M,
    pub tracking: T,
    pub localization: L,
    wheels: Option<Wheels>,
    track_width: Option<Length>,
    wheelbase: Option<Length>,
    driver: driver::DriverControl,
    last_driver: Option<Instant>,
    driver_output: bool,
    wheel_feedback: [Option<WheelFeedback>; 2],
}

impl<M: DrivetrainModel, T: Tracking, L> Drivetrain<M, T, L> {
    pub fn new(model: M, tracking: T, localization: L) -> Self {
        Self {
            model,
            tracking,
            localization,
            wheels: None,
            track_width: None,
            wheelbase: None,
            driver: driver::DriverControl::default(),
            last_driver: None,
            driver_output: false,
            wheel_feedback: [None; 2],
        }
    }

    pub fn with_track_width(mut self, width: Length) -> Result<Self, DriveConfigError> {
        positive(width.value, DriveConfigError::TrackWidth)?;
        self.track_width = Some(width);
        Ok(self)
    }

    pub fn with_wheelbase(mut self, length: Length) -> Result<Self, DriveConfigError> {
        positive(length.value, DriveConfigError::Wheelbase)?;
        self.wheelbase = Some(length);
        Ok(self)
    }

    pub fn track_width(&self) -> Option<Length> {
        self.track_width
    }
    pub fn wheelbase(&self) -> Option<Length> {
        self.wheelbase
    }
    pub fn wheel_diameter(&self) -> Option<Length> {
        self.wheels.as_ref().map(Wheels::diameter)
    }

    pub fn with_antitip(mut self, controller: AntiTip) -> Self {
        self.driver.antitip = Some(controller);
        self
    }

    pub fn with_headinghold(mut self, controller: HeadingHold) -> Self {
        self.driver.headinghold = Some(controller);
        self
    }
}

impl<M: DrivetrainModel + MotorGroups, T: Tracking, L> Drivetrain<M, T, L> {
    /// Set diameter and one ratio per motor, in model group and motor order.
    pub fn with_wheel_geometry(
        mut self,
        diameter: Length,
        gearing: &[&[WheelGearing]],
    ) -> Result<Self, DriveConfigError> {
        self.wheels = Some(Wheels::geared(
            diameter,
            self.model.motor_groups(),
            gearing,
        )?);
        Ok(self)
    }

    /// Infer ratios from the measured common free wheel RPM and each motor cartridge.
    pub fn with_wheel_feedback(
        mut self,
        diameter: Length,
        free_speed: AngularVelocity,
    ) -> Result<Self, DriveConfigError> {
        self.wheels = Some(Wheels::inferred(
            diameter,
            self.model.motor_groups(),
            free_speed,
        )?);
        Ok(self)
    }

    /// Apply physical wheel velocity targets in model group order.
    /// Conversion to motor RPM uses the same geometry as odometry.
    pub fn drive_wheel_speeds<const N: usize>(
        &mut self,
        speeds: [Velocity; N],
    ) -> Result<(), WheelSpeedError> {
        let groups = self.model.motor_groups_mut();
        let result = self
            .wheels
            .as_ref()
            .ok_or(WheelSpeedError::MissingWheelGeometry)
            .and_then(|wheels| wheels.drive_speed(groups, speeds));
        if matches!(
            result,
            Err(WheelSpeedError::InvalidCommand | WheelSpeedError::MissingWheelGeometry)
        ) {
            let _ = coast(groups);
        }
        result
    }

    pub fn wheel_load<const N: usize>(&self) -> [Option<WheelLoad>; N] {
        std::array::from_fn(|index| {
            self.wheels
                .as_ref()?
                .load(index, self.model.motor_groups().get(index)?)
        })
    }

    /// Apply absolute volts to each motor in groups, within their hardware limits.
    pub fn drive_volts<const N: usize>(
        &mut self,
        volts: [ElectricPotential; N],
    ) -> Result<(), PortError> {
        let groups = self.model.motor_groups_mut();
        if N != groups.len() || volts.iter().any(|voltage| !voltage.value.is_finite()) {
            return coast(groups);
        }
        groups
            .iter_mut()
            .zip(volts)
            .fold(Ok(()), |result, (group, voltage)| {
                result.and(each_motor(group, |motor| {
                    if voltage.value == 0.0 {
                        return motor.brake(BrakeMode::Coast);
                    }
                    let limit = motor.max_voltage();
                    motor.set_voltage(voltage.value.clamp(-limit, limit))
                }))
            })
    }
}

impl<T: Tracking, L> Drivetrain<Differential, T, L> {
    pub fn sample_feedback(&mut self) -> [Option<WheelFeedback>; 2] {
        self.wheel_feedback = self
            .wheels
            .as_mut()
            .map_or([None; 2], |wheels| wheels.sample(self.model.motor_groups()));
        self.wheel_feedback
    }

    /// Read the last measured translation and velocity without advancing odometry.
    pub fn feedback(&self) -> [Option<WheelFeedback>; 2] {
        self.wheel_feedback
    }

    pub fn measured_voltage(&self) -> [Option<ElectricPotential>; 2] {
        std::array::from_fn(|side| {
            self.model.motor_groups()[side]
                .first()?
                .voltage()
                .ok()
                .map(ElectricPotential::new::<ozton_math::uom::si::electric_potential::volt>)
        })
    }
}

impl<T: Tracking, L> DriveEncoders for Drivetrain<Differential, T, L> {
    fn track_width(&self) -> Option<Length> {
        self.track_width
    }
    fn tracking_positions(&mut self) -> [Option<Length>; 2] {
        self.sample_feedback()
            .map(|sample| sample.map(|sample| sample.position))
    }
}

/// Supply orientation for the drivetrain's driver assists.
pub trait DriverTracking: Tracking {
    fn driver_orientation(&mut self) -> (Option<Angle>, Option<Angle>);
}

impl DriverTracking for ozton_tracking::NoTracking {
    fn driver_orientation(&mut self) -> (Option<Angle>, Option<Angle>) {
        (None, None)
    }
}

impl DriverTracking for ozton_tracking::sampling::SensorTracking {
    fn driver_orientation(&mut self) -> (Option<Angle>, Option<Angle>) {
        let sample = self.refresh_orientation();
        (
            sample.and_then(|sample| sample.heading),
            sample.and_then(|sample| sample.roll),
        )
    }
}

impl DriverTracking for () {
    fn driver_orientation(&mut self) -> (Option<Angle>, Option<Angle>) {
        (None, None)
    }
}

impl<M: DrivetrainModel, T: Tracking, L> DrivetrainModel for Drivetrain<M, T, L> {
    type Error = M::Error;
}

impl<M: Tank, T: DriverTracking, L> Tank for Drivetrain<M, T, L> {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), Self::Error> {
        if vexide::competition::mode() == vexide::competition::CompetitionMode::Autonomous {
            return self.model.drive_tank(left, right);
        }
        let mut input = ArcadeInput::new((left + right) / 2.0, (left - right) / 2.0);
        if self.driver.headinghold.is_some() || self.driver.antitip.is_some() {
            let (heading, tilt) = self.tracking.driver_orientation();
            input = input.with_orientation(heading, tilt);
        }
        let now = Instant::now();
        let dt = self
            .last_driver
            .replace(now)
            .map_or(Duration::ZERO, |last| now.saturating_duration_since(last));
        let (forward, turn) = self.driver.apply(input, dt, 0.0);
        self.model.drive_tank(forward + turn, forward - turn)?;
        self.driver_output = true;
        Ok(())
    }

    fn stop_now(&mut self) -> Result<(), Self::Error> {
        self.driver.reset();
        self.last_driver = None;
        self.model.stop_now()
    }
}

impl<T: DriverTracking, L> Holonomic for Drivetrain<Mecanum, T, L> {
    fn drive_vector(&mut self, vector: glam::DVec2, turn: f64) -> Result<(), Self::Error> {
        if vexide::competition::mode() == vexide::competition::CompetitionMode::Autonomous {
            return self.model.drive_vector(vector, turn);
        }
        let (heading, tilt) = self.tracking.driver_orientation();
        let now = Instant::now();
        let dt = self
            .last_driver
            .replace(now)
            .map_or(Duration::ZERO, |last| now.saturating_duration_since(last));
        let (forward, turn) = self.driver.apply(
            ArcadeInput::new(vector.y, turn).with_orientation(heading, tilt),
            dt,
            vector.x,
        );
        self.model
            .drive_vector(glam::DVec2::new(vector.x, forward), turn)?;
        self.driver_output = true;
        Ok(())
    }
}

impl<T: DriverTracking, L> ProgramDrive for Drivetrain<Differential, T, L> {
    type Error = WheelSpeedError;
    type DriverCommand = ArcadeInput;
    type AutonCommand = [Velocity; 2];

    fn apply_driver(&mut self, command: ArcadeInput, _: Duration) -> Result<(), Self::Error> {
        use model::Arcade;
        let (throttle, turn) = command.scaled_axes();
        self.drive_arcade(throttle, turn)
            .map_err(WheelSpeedError::Port)
    }

    fn apply_auton(&mut self, speeds: Self::AutonCommand, _: Duration) -> Result<(), Self::Error> {
        self.drive_wheel_speeds(speeds)
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.stop_now().map_err(WheelSpeedError::Port)
    }

    fn take_driver_output(&mut self) -> bool {
        std::mem::take(&mut self.driver_output)
    }
}

impl<T: DriverTracking, L> ProgramDrive for Drivetrain<Mecanum, T, L> {
    type Error = WheelSpeedError;
    type DriverCommand = (f64, ArcadeInput);
    type AutonCommand = [Velocity; 4];

    fn apply_driver(
        &mut self,
        (strafe, input): Self::DriverCommand,
        _: Duration,
    ) -> Result<(), Self::Error> {
        self.drive_vector(
            glam::DVec2::new(strafe, input.throttle) * input.output_scale,
            input.turn * input.output_scale,
        )
        .map_err(WheelSpeedError::Port)
    }

    fn apply_auton(&mut self, speeds: Self::AutonCommand, _: Duration) -> Result<(), Self::Error> {
        self.drive_wheel_speeds(speeds)
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.stop_now().map_err(WheelSpeedError::Port)
    }

    fn take_driver_output(&mut self) -> bool {
        std::mem::take(&mut self.driver_output)
    }
}

pub(crate) fn each_motor(
    motors: &mut [Motor],
    mut command: impl FnMut(&mut Motor) -> Result<(), PortError>,
) -> Result<(), PortError> {
    motors
        .iter_mut()
        .fold(Ok(()), |result, motor| result.and(command(motor)))
}

pub(crate) fn coast(groups: &mut [Box<[Motor]>]) -> Result<(), PortError> {
    groups.iter_mut().fold(Ok(()), |result, group| {
        result.and(each_motor(group, |motor| motor.brake(BrakeMode::Coast)))
    })
}
