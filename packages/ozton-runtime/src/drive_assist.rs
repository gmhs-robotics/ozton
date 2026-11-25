//! Inline drive assists and the borrowed drivetrain interface.

use std::time::{Duration, Instant};

use ozton_drivetrain::{
    BrakeMode, BrakeMotors, Drivetrain, HasBraking, HasLocalization, MotorAddress, MotorGroups,
    WheelActuation, WheelCommand, WheelSpeedError,
    model::{Drive, LateralDrive, tank_axes},
};
use ozton_math::{
    desaturate,
    geometry::DVec2,
    uom::si::{
        f64::{Length, Velocity},
        velocity::meter_per_second,
    },
};
use ozton_tracking::{HasDriveWheels, TrackingSystem, odometric::DriveEncoders};
use vexide::smart::PortError;

mod collection;
mod execution;
pub use collection::{DriveAssists, ResetDriveAssists};
pub use ozton_drivetrain::assist::{AssistContext, DriveAssist, MotorBalanceAssist};

const MAX_CONTROL_GAP: Duration = Duration::from_millis(100);

/// A temporary drive view over an assembled drivetrain and its installed assists.
pub struct AssistedDrivetrain<'a, M: Drive, T: TrackingSystem, L, A> {
    drivetrain: &'a mut Drivetrain<M, T, L>,
    assists: &'a mut DriveAssists<A>,
}

impl<'a, M: Drive, T: TrackingSystem, L, A: DriveAssist<M, T, L>>
    AssistedDrivetrain<'a, M, T, L, A>
{
    pub fn new(drivetrain: &'a mut Drivetrain<M, T, L>, assists: &'a mut DriveAssists<A>) -> Self {
        Self {
            drivetrain,
            assists,
        }
    }
}

impl<'a, M: Drive, T: TrackingSystem, L, A> AssistedDrivetrain<'a, M, T, L, A> {
    pub fn drivetrain(&self) -> &Drivetrain<M, T, L> {
        self.drivetrain
    }

    pub fn drivetrain_mut(&mut self) -> &mut Drivetrain<M, T, L> {
        self.drivetrain
    }
}

impl<M: Drive, T: TrackingSystem, L, A: DriveAssist<M, T, L>> AssistedDrivetrain<'_, M, T, L, A> {
    fn mix(
        &mut self,
        mut axes: [f64; 2],
        wheel_velocity: bool,
    ) -> ([f64; 2], Result<(), M::Error>) {
        let now = Instant::now();
        let elapsed = match self.assists.last_tick.replace((now, wheel_velocity)) {
            Some((last, previous))
                if previous == wheel_velocity && now - last <= MAX_CONTROL_GAP =>
            {
                now - last
            }
            _ => {
                self.assists.items.reset();
                Duration::ZERO
            }
        };
        let tracking = self.drivetrain.tracking();
        let localization = self.drivetrain.localization();
        let context = AssistContext {
            model: &self.drivetrain.model,
            tracking: &*tracking,
            localization: &*localization,
            elapsed,
            wheel_velocity,
        };
        self.assists.items.apply(&mut axes, &context);
        let mut wheels = [axes[0] + axes[1], axes[0] - axes[1]];
        let result = self.assists.items.wheels(&context, &mut wheels);
        (wheels, result)
    }

    fn reset(&mut self) {
        self.assists.last_tick = None;
        self.assists.items.reset();
    }
}

impl<M: Drive, T: TrackingSystem, L, A: DriveAssist<M, T, L>> Drive
    for AssistedDrivetrain<'_, M, T, L, A>
{
    type Error = M::Error;

    fn drive_vector(&mut self, mut vector: DVec2, turn: f64) -> Result<(), Self::Error> {
        if !vector.is_finite() || !turn.is_finite() {
            self.reset();
            return self.drivetrain.drive_vector(DVec2::ZERO, 0.0);
        }
        let (wheels, adjustment) = self.mix([vector.y, turn], false);
        let (forward, turn) = tank_axes(wheels);
        vector.y = forward;
        self.drivetrain.drive_vector(vector, turn).and(adjustment)
    }

    fn stop_now(&mut self) -> Result<(), Self::Error> {
        self.reset();
        self.drivetrain.stop_now()
    }
}

impl<M: LateralDrive, T: TrackingSystem, L, A: DriveAssist<M, T, L>> LateralDrive
    for AssistedDrivetrain<'_, M, T, L, A>
{
}

impl<M: Drive, T: TrackingSystem, L, A> HasLocalization for AssistedDrivetrain<'_, M, T, L, A>
where
    Drivetrain<M, T, L>: HasLocalization<Localization = L>,
{
    type Localization = L;

    fn localization(&self) -> impl std::ops::Deref<Target = L> {
        self.drivetrain.localization()
    }

    fn localization_mut(&mut self) -> impl std::ops::DerefMut<Target = L> {
        self.drivetrain.localization_mut()
    }
}

impl<M: Drive, T: TrackingSystem, L, A> DriveEncoders for AssistedDrivetrain<'_, M, T, L, A>
where
    Drivetrain<M, T, L>: DriveEncoders,
{
    fn track_width(&self) -> Length {
        DriveEncoders::track_width(self.drivetrain)
    }

    fn tracking_positions(&mut self) -> [Option<Length>; 2] {
        self.drivetrain.tracking_positions()
    }
}

impl<
    M: Drive<Error = PortError> + MotorGroups<WheelSpeeds = [Velocity; 2]>,
    T: HasDriveWheels,
    L,
    A: DriveAssist<M, T, L>,
> AssistedDrivetrain<'_, M, T, L, A>
{
    /// Apply assists to a two-sided physical wheel velocity command.
    pub fn drive_wheel_speeds(&mut self, speeds: [Velocity; 2]) -> Result<(), WheelSpeedError> {
        self.command_wheels(speeds.map(WheelCommand::Velocity))
    }

    /// Preserve explicit stops while applying the ordinary drive assists to
    /// moving wheels. A stopped side cannot be reactivated by an assist.
    pub fn command_wheels(&mut self, commands: [WheelCommand; 2]) -> Result<(), WheelSpeedError> {
        let speeds = commands.map(|command| command.velocity().get::<meter_per_second>());
        let scale = speeds[0].abs().max(speeds[1].abs());
        if !commands.iter().all(|command| command.is_valid()) || scale == 0.0 {
            self.reset();
            return self.drivetrain.command_wheels(commands);
        }
        let (forward, turn) = tank_axes(speeds.map(|speed| speed / scale));
        let (wheels, adjustment) = self.mix([forward, turn], true);
        let wheels = desaturate(wheels, 1.0);
        let applied = core::array::from_fn(|side| match commands[side] {
            WheelCommand::Brake(mode) => WheelCommand::Brake(mode),
            WheelCommand::Velocity(_) => {
                WheelCommand::Velocity(Velocity::new::<meter_per_second>(wheels[side] * scale))
            }
        });
        self.drivetrain
            .command_wheels(applied)
            .and(adjustment.map_err(WheelSpeedError::Port))
    }
}

impl<M, T, L, A> HasBraking for AssistedDrivetrain<'_, M, T, L, A>
where
    M: Drive + HasBraking,
    T: TrackingSystem,
    A: DriveAssist<M, T, L>,
{
    type BrakeError = M::BrakeError;

    fn apply_brake(&mut self, mode: BrakeMode) -> Result<(), Self::BrakeError> {
        self.reset();
        self.drivetrain.apply_brake(mode)
    }
}

impl<M, T, L, A> BrakeMotors for AssistedDrivetrain<'_, M, T, L, A>
where
    M: Drive + BrakeMotors,
    T: TrackingSystem,
    A: DriveAssist<M, T, L>,
{
    fn brake_motors(
        &mut self,
        select: impl FnMut(MotorAddress) -> Option<BrakeMode>,
    ) -> Result<(), Self::BrakeError> {
        self.drivetrain.brake_motors(select)
    }
}

impl<M, T, L, A> WheelActuation for AssistedDrivetrain<'_, M, T, L, A>
where
    M: Drive<Error = PortError> + HasBraking + MotorGroups<WheelSpeeds = [Velocity; 2]>,
    T: HasDriveWheels,
    A: DriveAssist<M, T, L>,
{
    type WheelError = WheelSpeedError;

    fn drive_wheel_commands(
        &mut self,
        commands: [WheelCommand; 2],
    ) -> Result<(), Self::WheelError> {
        self.command_wheels(commands)
    }
}

#[cfg(test)]
mod tests;
