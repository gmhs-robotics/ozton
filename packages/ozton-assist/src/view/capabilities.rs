//! Preserve optional capabilities of the underlying drive.
use std::{
    future::Future,
    ops::{Deref, DerefMut},
    time::{Duration, Instant},
};

use ozton_drivetrain::{
    BrakeMode, BrakeMotors, Drive, Footprint, HasBraking, HasLocalization, MotionExecution,
    MotorAddress, RangeResetDrive, WheelActuation, WheelCommand, model::DriveEncoders,
};
use ozton_math::uom::si::f64::{Length, Velocity};

use super::{AssistedDrive, DriveAssist};

impl<D: HasLocalization + ?Sized, A> HasLocalization for AssistedDrive<'_, D, A> {
    type Localization = D::Localization;

    fn localization(&self) -> impl Deref<Target = Self::Localization> {
        self.drive.localization()
    }

    fn localization_mut(&mut self) -> impl DerefMut<Target = Self::Localization> {
        self.drive.localization_mut()
    }
}

impl<D: DriveEncoders + ?Sized, A> DriveEncoders for AssistedDrive<'_, D, A> {
    fn track_width(&self) -> Length {
        self.drive.track_width()
    }

    fn tracking_positions(&mut self) -> [Option<Length>; 2] {
        self.drive.tracking_positions()
    }
}

impl<D: Drive + HasBraking + ?Sized, A: DriveAssist<D>> HasBraking for AssistedDrive<'_, D, A> {
    type BrakeError = D::BrakeError;

    fn apply_brake(&mut self, mode: BrakeMode) -> Result<(), Self::BrakeError> {
        self.reset();
        self.drive.apply_brake(mode)
    }
}

impl<D: Drive + BrakeMotors + ?Sized, A: DriveAssist<D>> BrakeMotors for AssistedDrive<'_, D, A> {
    fn brake_motors(
        &mut self,
        select: impl FnMut(MotorAddress) -> Option<BrakeMode>,
    ) -> Result<(), Self::BrakeError> {
        self.drive.brake_motors(select)
    }
}

impl<D, A> WheelActuation for AssistedDrive<'_, D, A>
where
    D: Drive + WheelActuation<WheelError: From<D::Error>> + ?Sized,
    A: DriveAssist<D>,
{
    type WheelError = D::WheelError;

    fn drive_wheel_commands(
        &mut self,
        commands: [WheelCommand; 2],
    ) -> Result<(), Self::WheelError> {
        let (speeds, adjustment) = self.mix_velocity(commands.map(WheelCommand::velocity));
        let applied = core::array::from_fn(|side| match commands[side] {
            WheelCommand::Brake(mode) => WheelCommand::Brake(mode),
            WheelCommand::Velocity(_) => WheelCommand::Velocity(speeds[side]),
        });
        self.drive
            .drive_wheel_commands(applied)
            .and(adjustment.map_err(Into::into))
    }
}

impl<D, A> MotionExecution for AssistedDrive<'_, D, A>
where
    D: Drive + MotionExecution<Error: From<<D as Drive>::Error>> + ?Sized,
    A: DriveAssist<D>,
{
    type Error = <D as MotionExecution>::Error;

    fn footprint(&self) -> Option<Footprint> {
        self.drive.footprint()
    }

    fn drive_wheels(&mut self, speeds: [Velocity; 2]) -> Result<(), Self::Error> {
        let (speeds, adjustment) = self.mix_velocity(speeds);
        self.drive
            .drive_wheels(speeds)
            .and(adjustment.map_err(Into::into))
    }

    fn brake(&mut self) -> Result<(), Self::Error> {
        self.reset();
        self.drive.brake()
    }

    fn emergency_brake(&mut self) -> Result<(), Self::Error> {
        self.reset();
        self.drive.emergency_brake()
    }

    fn wait_until(&mut self, deadline: Instant) -> impl Future<Output = ()> {
        self.drive.wait_until(deadline)
    }

    fn motion_active(&self) -> bool {
        self.drive.motion_active()
    }

    fn log_step(&mut self, elapsed: Duration, dt: Duration) {
        self.drive.log_step(elapsed, dt);
    }
}

impl<D, A> RangeResetDrive for AssistedDrive<'_, D, A>
where
    D: Drive + RangeResetDrive<Error: From<<D as Drive>::Error>> + ?Sized,
    A: DriveAssist<D>,
{
}
