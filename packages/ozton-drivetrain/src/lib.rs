//! A motion model, the robot's physical geometry, and its tracking system.
use model::Differential;
pub use model::{Drive, LateralDrive};
pub use ozton_math::path::clearance::Footprint;
use ozton_math::uom::si::f64::{ElectricPotential, Length, Velocity};
pub use ozton_tracking::odometric::{DriveConfigError, SharedMotors, WheelFeedback, WheelGearing};
use ozton_tracking::{
    TrackingSystem,
    odometric::{DifferentialTracking, DriveEncoders},
    sampling::LocalizationLifecycle,
};
use vexide::{
    prelude::Motor,
    smart::{PortError, motor::BrakeMode},
};
pub use wheels::WheelSpeedError;

pub mod model;
mod wheels;

/// Access the model's motor groups in their fixed wheel order.
pub trait MotorGroups {
    /// Physical wheel-speed commands, one entry per group in model order.
    type WheelSpeeds: AsRef<[Velocity]>;

    fn motor_groups(&self) -> &[SharedMotors];
    fn motor_groups_mut(&mut self) -> &mut [SharedMotors];
}

/// A drivetrain owns geometry and feedback calibration independently of how it mixes motor outputs.
pub struct Drivetrain<M: Drive, T: TrackingSystem, L> {
    pub model: M,
    pub tracking: T,
    pub localization: L,
    driver_output: bool,
}

impl<M: Drive, T: TrackingSystem, L> Drivetrain<M, T, L> {
    pub fn new(model: M, tracking: T, localization: L) -> Self {
        Self {
            model,
            tracking,
            localization,
            driver_output: false,
        }
    }

    /// Consume the write marker used by competition output watchdogs.
    pub fn take_output(&mut self) -> bool {
        std::mem::take(&mut self.driver_output)
    }

    pub fn track_width(&self) -> Option<Length> {
        self.tracking
            .drive_wheels()
            .map(DifferentialTracking::track_width)
    }

    pub fn wheel_diameter(&self) -> Option<Length> {
        self.tracking
            .drive_wheels()
            .map(|source| source.wheels().diameter())
    }
}

impl<M: Drive + MotorGroups, T: TrackingSystem, L> Drivetrain<M, T, L> {
    /// Apply physical wheel velocity targets in model group order.
    /// Conversion to motor RPM uses the same geometry as odometry.
    pub fn drive_wheel_speeds<const N: usize>(
        &mut self,
        speeds: [Velocity; N],
    ) -> Result<(), WheelSpeedError> {
        self.set_wheel_speeds(&speeds)
    }

    /// Apply a slice of physical wheel speeds in model group order.
    pub fn set_wheel_speeds(&mut self, speeds: &[Velocity]) -> Result<(), WheelSpeedError> {
        let groups = self.model.motor_groups_mut();
        let result = self
            .tracking
            .drive_wheels()
            .map(DifferentialTracking::wheels)
            .ok_or(WheelSpeedError::MissingWheelGeometry)
            .and_then(|wheels| wheels::drive_speed(wheels, groups, speeds));
        if matches!(
            result,
            Err(WheelSpeedError::InvalidCommand | WheelSpeedError::MissingWheelGeometry)
        ) {
            let _ = coast(groups);
        }
        result
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
        command_groups(groups.iter_mut().zip(volts), |motor, voltage| {
            if voltage.value == 0.0 {
                return motor.brake(BrakeMode::Coast);
            }
            let limit = motor.max_voltage();
            motor.set_voltage(voltage.value.clamp(-limit, limit))
        })
    }
}

impl<M: Drive + MotorGroups, T: TrackingSystem, L> Drivetrain<M, T, L> {
    pub fn sample_feedback(&mut self) -> [Option<WheelFeedback>; 2] {
        self.tracking
            .drive_wheels_mut()
            .map_or([None; 2], DifferentialTracking::read_wheels)
    }

    /// Read the last measured translation and velocity without advancing odometry.
    pub fn feedback(&self) -> [Option<WheelFeedback>; 2] {
        self.tracking
            .drive_wheels()
            .map_or([None; 2], DifferentialTracking::feedback)
    }
}

impl<T: TrackingSystem, L> Drivetrain<Differential, T, L> {
    pub fn footprint(&self) -> Option<Footprint> {
        self.model.footprint()
    }

    pub fn measured_voltage(&self) -> [Option<ElectricPotential>; 2] {
        std::array::from_fn(|side| {
            self.model.motor_groups()[side]
                .borrow()
                .first()?
                .voltage()
                .ok()
                .map(ElectricPotential::new::<ozton_math::uom::si::electric_potential::volt>)
        })
    }
}

impl<T: TrackingSystem, L> DriveEncoders for Drivetrain<Differential, T, L> {
    fn track_width(&self) -> Option<Length> {
        Drivetrain::track_width(self)
    }
    fn tracking_positions(&mut self) -> [Option<Length>; 2] {
        self.sample_feedback()
            .map(|sample| sample.map(|sample| sample.position))
    }
}

/// The localization estimate exposed by a drive.
pub trait HasLocalization {
    type Localization;

    fn localization(&self) -> &Self::Localization;

    /// Sample the drive before a control step. Externally updated estimates can use the default.
    fn refresh_localization(&mut self) {}
}

impl<M: Drive + MotorGroups, T: TrackingSystem, L: LocalizationLifecycle<T>> HasLocalization
    for Drivetrain<M, T, L>
{
    type Localization = L;

    fn localization(&self) -> &L {
        &self.localization
    }

    fn refresh_localization(&mut self) {
        self.localization.poll(&mut self.tracking);
    }
}

impl<M: Drive, T: TrackingSystem, L> Drive for Drivetrain<M, T, L> {
    type Error = M::Error;

    fn drive_vector(&mut self, vector: glam::DVec2, turn: f64) -> Result<(), Self::Error> {
        self.driver_output = true;
        self.model.drive_vector(vector, turn)
    }

    fn stop_now(&mut self) -> Result<(), Self::Error> {
        self.model.stop_now()
    }
}

impl<M: LateralDrive, T: TrackingSystem, L> LateralDrive for Drivetrain<M, T, L> {}

pub(crate) fn normalized_voltage<const N: usize>(
    groups: &mut [SharedMotors; N],
    powers: [f64; N],
) -> Result<(), PortError> {
    if powers.iter().any(|power| !power.is_finite()) {
        return coast(groups);
    }
    command_groups(
        groups.iter_mut().zip(ozton_math::desaturate(powers, 1.0)),
        |motor, power| {
            if power == 0.0 {
                motor.brake(BrakeMode::Coast)
            } else {
                motor.set_voltage(power * motor.max_voltage())
            }
        },
    )
}

fn command_groups<'a, T: Copy>(
    groups: impl IntoIterator<Item = (&'a mut SharedMotors, T)>,
    mut command: impl FnMut(&mut Motor, T) -> Result<(), PortError>,
) -> Result<(), PortError> {
    let mut result = Ok(());
    for (group, input) in groups {
        result = result.and(each_motor(&mut group.borrow_mut(), |motor| {
            command(motor, input)
        }));
    }
    result
}

pub(crate) fn each_motor(
    motors: &mut [Motor],
    mut command: impl FnMut(&mut Motor) -> Result<(), PortError>,
) -> Result<(), PortError> {
    // Every motor must receive the command, including after an earlier failure.
    let mut result = Ok(());
    for motor in motors {
        result = result.and(command(motor));
    }
    result
}

pub(crate) fn coast(groups: &mut [SharedMotors]) -> Result<(), PortError> {
    command_groups(groups.iter_mut().zip(std::iter::repeat(())), |motor, ()| {
        motor.brake(BrakeMode::Coast)
    })
}
