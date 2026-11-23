//! Command assists and the assisted drivetrain view.

use std::time::{Duration, Instant};

use ozton_tracking::{NoTracking, Tracking, sampling::SensorTracking, sources::vex::Orientation};

use crate::{
    Drivetrain,
    model::{ArcadeInput, Balancing, DrivetrainModel, Tank, arcade_mix},
};

mod collection;
pub use collection::DriveAssists;

const MAX_CONTROL_GAP: Duration = Duration::from_millis(100);

/// An assist transforms arcade inputs or mixed wheel commands before the single motor write.
pub trait DriveAssist<M: DrivetrainModel> {
    fn needs_orientation(&self) -> bool {
        false
    }

    fn apply(&mut self, _axes: &mut [f64; 2], _orientation: Option<Orientation>, _dt: Duration) {}

    fn wheels(&mut self, _model: &M, _wheels: &mut [f64; 2]) -> Result<(), M::Error> {
        Ok(())
    }

    fn reset(&mut self) {}
}

impl<M: DrivetrainModel> DriveAssist<M> for () {}

/// Two assists in installation order. The chain is stored inline.
pub struct AssistChain<A, B>(pub(crate) A, pub(crate) B);

impl<M: DrivetrainModel, A: DriveAssist<M>, B: DriveAssist<M>> DriveAssist<M>
    for AssistChain<A, B>
{
    fn needs_orientation(&self) -> bool {
        self.0.needs_orientation() || self.1.needs_orientation()
    }

    fn apply(&mut self, axes: &mut [f64; 2], orientation: Option<Orientation>, dt: Duration) {
        self.0.apply(axes, orientation, dt);
        self.1.apply(axes, orientation, dt);
    }

    fn wheels(&mut self, model: &M, wheels: &mut [f64; 2]) -> Result<(), M::Error> {
        self.0
            .wheels(model, wheels)
            .and(self.1.wheels(model, wheels))
    }

    fn reset(&mut self) {
        self.0.reset();
        self.1.reset();
    }
}

/// Use the drivetrain model's motor-loss policy before writing wheel outputs.
#[derive(Clone, Copy, Default)]
pub struct MotorBalanceAssist;

impl<M: Balancing<Command = [f64; 2]>> DriveAssist<M> for MotorBalanceAssist {
    fn wheels(&mut self, model: &M, wheels: &mut [f64; 2]) -> Result<(), M::Error> {
        model.balance(wheels)
    }
}

/// Orientation source for installed assists. A drivetrain without an IMU can still use assists.
pub trait AssistTracking: Tracking {
    fn assist_orientation(&mut self) -> Option<Orientation> {
        None
    }
}

impl AssistTracking for () {}
impl AssistTracking for NoTracking {}

impl AssistTracking for SensorTracking {
    fn assist_orientation(&mut self) -> Option<Orientation> {
        self.refresh_orientation()
    }
}

/// A borrowed drive interface that applies the installed assists.
pub struct AssistedDrivetrain<'a, M: DrivetrainModel, T: Tracking, L, A> {
    pub(crate) drive: &'a mut Drivetrain<M, T, L, A>,
}

impl<'a, M: DrivetrainModel, T: Tracking, L, A> AssistedDrivetrain<'a, M, T, L, A> {
    pub fn into_inner(self) -> &'a mut Drivetrain<M, T, L, A> {
        self.drive
    }
}

impl<M: Tank, T: AssistTracking, L, A: DriveAssist<M>> AssistedDrivetrain<'_, M, T, L, A> {
    /// Issue a driver command. Ozton's competition loop reports any output fault.
    pub fn drive(&mut self, input: ArcadeInput) {
        if let Err(error) = self.try_drive(input) {
            self.drive.output_fault.get_or_insert(error);
        }
    }

    /// Issue a command and return its error, for motion routines that must abort on output faults.
    pub fn try_drive(&mut self, input: ArcadeInput) -> Result<(), M::Error> {
        let drive = &mut self.drive;
        let now = Instant::now();
        let dt = drive
            .assist_tick
            .replace(now)
            .map_or(Duration::ZERO, |last| now - last);
        if dt > MAX_CONTROL_GAP {
            drive.assists.reset();
        }
        let (forward, turn) = input.scaled_axes();
        if !forward.is_finite() || !turn.is_finite() {
            drive.driver_output = true;
            return drive.stop_now();
        }
        let orientation = drive
            .assists
            .needs_orientation()
            .then(|| drive.tracking.assist_orientation())
            .flatten();
        let mut axes = [forward, turn];
        drive
            .assists
            .apply(&mut axes, orientation, dt.min(MAX_CONTROL_GAP));
        let mut outputs = arcade_mix(axes[0], axes[1]);
        let balance = drive.assists.wheels(&drive.model, &mut outputs);
        drive.drive_tank(outputs[0], outputs[1]).and(balance)
    }

    pub fn stop(&mut self) -> Result<(), M::Error> {
        self.drive.stop_now()
    }
}

#[cfg(test)]
mod tests;
