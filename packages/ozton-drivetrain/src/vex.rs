//! VEX drivetrain actuation and tracking integration.
use std::{
    cell::{Cell, Ref, RefCell, RefMut},
    rc::Rc,
    time::Duration,
};

use ozton_math::uom::si::f64::{ElectricPotential, Length, Velocity};
use ozton_tracking::{
    HasDriveWheels, TrackingSystem,
    odometric::{DriveEncoders, SharedMotors, WheelFeedback},
};
use vexide::{
    prelude::Motor,
    smart::{PortError, motor::BrakeMode},
};
use wheels::WheelSpeedError;

use super::{
    BrakeMotors, Drive, Footprint, HasBraking, HasLocalization, LateralDrive, MotorAddress,
    WheelCommand,
    wheels,
};
use crate::model::{Differential, MotionGeometry};

/// Access the model's motor groups in their fixed wheel order.
pub trait MotorGroups {
    /// Physical wheel-speed commands, one entry per group in model order.
    type WheelSpeeds: AsRef<[Velocity]>;
    /// Voltage commands in the same model order.
    type WheelVoltages: AsRef<[ElectricPotential]>;

    fn motor_groups(&self) -> &[SharedMotors];
    fn motor_groups_mut(&mut self) -> &mut [SharedMotors];
}

/// A drivetrain owns geometry and feedback calibration independently of how it mixes motor outputs.
pub struct Drivetrain<M: Drive, T = (), L = ()> {
    pub model: M,
    state: Rc<RefCell<(T, L)>>,
    driver_output: bool,
    diagnostic_stamp: Rc<Cell<Option<Duration>>>,
}

impl<M: Drive, T, L> Drivetrain<M, T, L> {
    pub fn new(model: M, tracking: T, localization: L) -> Self {
        Self {
            model,
            state: Rc::new(RefCell::new((tracking, localization))),
            driver_output: false,
            diagnostic_stamp: Rc::new(Cell::new(None)),
        }
    }

    /// Shared rate gate for independently scheduled diagnostic reporters.
    #[doc(hidden)]
    pub fn diagnostic_stamp(&self) -> Rc<Cell<Option<Duration>>> {
        Rc::clone(&self.diagnostic_stamp)
    }

    /// Consume the write marker used by competition output watchdogs.
    pub fn take_output(&mut self) -> bool {
        std::mem::take(&mut self.driver_output)
    }

    pub fn tracking(&self) -> Ref<'_, T> {
        Ref::map(self.state.borrow(), |(tracking, _)| tracking)
    }

    pub fn tracking_mut(&mut self) -> RefMut<'_, T> {
        RefMut::map(self.state.borrow_mut(), |(tracking, _)| tracking)
    }

    pub fn localization(&self) -> Ref<'_, L> {
        Ref::map(self.state.borrow(), |(_, localization)| localization)
    }

    pub fn localization_mut(&mut self) -> RefMut<'_, L> {
        RefMut::map(self.state.borrow_mut(), |(_, localization)| localization)
    }

    /// Share the observation sources and estimator with a runtime sampling task.
    #[doc(hidden)]
    pub fn localization_state(&self) -> std::rc::Weak<RefCell<(T, L)>> {
        Rc::downgrade(&self.state)
    }
}

impl<M: Drive, T: HasDriveWheels, L> Drivetrain<M, T, L> {
    pub fn track_width(&self) -> Length {
        self.tracking().calibrated_drive().track_width()
    }

    pub fn wheel_diameter(&self) -> Length {
        self.tracking().calibrated_drive().wheels().diameter()
    }
}

impl<M: Drive + MotorGroups<WheelSpeeds = [Velocity; 2]>, T: HasDriveWheels, L>
    Drivetrain<M, T, L>
{
    /// Apply physical wheel velocity targets in model group order.
    /// Conversion to motor RPM uses the same geometry as odometry.
    pub fn drive_wheel_speeds(&mut self, speeds: M::WheelSpeeds) -> Result<(), WheelSpeedError> {
        self.command_wheels(speeds.map(WheelCommand::Velocity))
    }

    /// Apply calibrated velocities and explicit braking modes in group order.
    pub fn command_wheels(&mut self, commands: [WheelCommand; 2]) -> Result<(), WheelSpeedError> {
        let groups = self.model.motor_groups_mut();
        let state = self.state.borrow();
        let result = wheels::drive_commands(state.0.calibrated_drive().wheels(), groups, &commands);
        if matches!(result, Err(WheelSpeedError::InvalidCommand)) {
            let _ = coast(groups);
        }
        result
    }
}

impl From<crate::BrakeMode> for BrakeMode {
    fn from(mode: crate::BrakeMode) -> Self {
        match mode {
            crate::BrakeMode::Coast => Self::Coast,
            crate::BrakeMode::Brake => Self::Brake,
            crate::BrakeMode::Hold => Self::Hold,
        }
    }
}

impl HasBraking for Motor {
    type BrakeError = PortError;

    fn apply_brake(&mut self, mode: crate::BrakeMode) -> Result<(), Self::BrakeError> {
        self.brake(mode.into())
    }
}

impl<M: Drive + HasBraking, T, L> HasBraking for Drivetrain<M, T, L> {
    type BrakeError = M::BrakeError;

    fn apply_brake(&mut self, mode: crate::BrakeMode) -> Result<(), Self::BrakeError> {
        self.model.apply_brake(mode)
    }
}

impl<M: Drive + BrakeMotors, T, L> BrakeMotors for Drivetrain<M, T, L> {
    fn brake_motors(
        &mut self,
        select: impl FnMut(MotorAddress) -> Option<crate::BrakeMode>,
    ) -> Result<(), Self::BrakeError> {
        self.model.brake_motors(select)
    }
}

impl<M: Drive + MotorGroups, T, L> Drivetrain<M, T, L> {
    /// Apply absolute volts to each motor in groups, within their hardware limits.
    pub fn drive_volts(&mut self, volts: M::WheelVoltages) -> Result<(), PortError> {
        let groups = self.model.motor_groups_mut();
        let volts = volts.as_ref();
        if volts.len() != groups.len() || volts.iter().any(|voltage| !voltage.value.is_finite()) {
            return coast(groups);
        }
        command_groups(
            groups.iter_mut().zip(volts.iter().copied()),
            |motor, voltage| {
                if voltage.value == 0.0 {
                    return motor.brake(BrakeMode::Coast);
                }
                let limit = motor.max_voltage();
                motor.set_voltage(voltage.value.clamp(-limit, limit))
            },
        )
    }
}

impl<M: Drive + MotorGroups, T: HasDriveWheels, L> Drivetrain<M, T, L> {
    pub fn sample_feedback(&mut self) -> [Option<WheelFeedback>; 2] {
        self.tracking_mut().calibrated_drive_mut().read_wheels()
    }

    /// Read the last measured translation and velocity without advancing odometry.
    pub fn feedback(&self) -> [Option<WheelFeedback>; 2] {
        self.tracking().calibrated_drive().feedback()
    }
}

impl<G: MotionGeometry, T: TrackingSystem, L> Drivetrain<Differential<G>, T, L> {
    pub fn footprint(&self) -> Option<Footprint> {
        self.model.footprint()
    }
}

impl<G, T: TrackingSystem, L> Drivetrain<Differential<G>, T, L> {
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

impl<G, T: HasDriveWheels, L> DriveEncoders for Drivetrain<Differential<G>, T, L> {
    fn track_width(&self) -> Length {
        self.tracking().calibrated_drive().track_width()
    }
    fn tracking_positions(&mut self) -> [Option<Length>; 2] {
        self.tracking_mut()
            .calibrated_drive_mut()
            .read_wheels()
            .map(|sample| sample.map(|sample| sample.position))
    }
}

impl<M: Drive, T, L> HasLocalization for Drivetrain<M, T, L> {
    type Localization = L;

    fn localization(&self) -> impl std::ops::Deref<Target = L> {
        Drivetrain::localization(self)
    }
}

impl<M: Drive, T, L> Drive for Drivetrain<M, T, L> {
    type Error = M::Error;

    fn drive_vector(&mut self, vector: glam::DVec2, turn: f64) -> Result<(), Self::Error> {
        self.driver_output = true;
        self.model.drive_vector(vector, turn)
    }

    fn stop_now(&mut self) -> Result<(), Self::Error> {
        self.model.stop_now()
    }
}

impl<M: LateralDrive, T, L> LateralDrive for Drivetrain<M, T, L> {}

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
