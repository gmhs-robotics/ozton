//! Inline drive assists and the borrowed drivetrain interface.

use std::time::{Duration, Instant};

use ozton_drivetrain::{
    Drivetrain, HasLocalization, MotorGroups, WheelSpeedError,
    model::{Balancing, Drive, LateralDrive, tank_axes},
};
use ozton_math::{
    desaturate,
    geometry::DVec2,
    uom::si::{
        f64::{Length, Velocity},
        velocity::meter_per_second,
    },
};
use ozton_tracking::{
    NoTracking, Tracking, TrackingSystem, inertial::Orientation, odometric::DriveEncoders,
};
use vexide::smart::PortError;

mod collection;
pub use collection::{DriveAssists, ResetDriveAssists};

const MAX_CONTROL_GAP: Duration = Duration::from_millis(100);

/// Observations and estimates borrowed from the drive for a single command.
pub struct AssistContext<'a, M, T, L> {
    pub model: &'a M,
    pub tracking: &'a T,
    pub localization: &'a L,
    pub orientation: Option<Orientation>,
    pub elapsed: Duration,
    /// Whether the caller supplies physical wheel velocity targets.
    pub wheel_velocity: bool,
}

/// Transform forward/turn axes or adjust the resulting side commands.
pub trait DriveAssist<M: Drive, T, L> {
    fn needs_orientation(&self) -> bool {
        false
    }

    fn apply(&mut self, _axes: &mut [f64; 2], _context: &AssistContext<'_, M, T, L>) {}

    fn wheels(
        &mut self,
        _context: &AssistContext<'_, M, T, L>,
        _wheels: &mut [f64; 2],
    ) -> Result<(), M::Error> {
        Ok(())
    }

    fn reset(&mut self) {}
}

impl<M: Drive, T, L> DriveAssist<M, T, L> for () {}

/// Use the drivetrain model's motor-loss policy before writing outputs.
#[derive(Clone, Copy, Default)]
pub struct MotorBalanceAssist;

impl<M: Balancing, T, L> DriveAssist<M, T, L> for MotorBalanceAssist {
    fn wheels(
        &mut self,
        context: &AssistContext<'_, M, T, L>,
        wheels: &mut [f64; 2],
    ) -> Result<(), M::Error> {
        *wheels = desaturate(*wheels, 1.0);
        context.model.balance(wheels)
    }
}

/// Hardware observation used by assists that need orientation.
pub trait AssistTracking: TrackingSystem {
    fn assist_orientation(&mut self) -> Option<Orientation> {
        None
    }
}

impl AssistTracking for () {}
impl AssistTracking for NoTracking {}

impl AssistTracking for Tracking {
    fn assist_orientation(&mut self) -> Option<Orientation> {
        self.refresh_orientation()
    }
}

/// A temporary drive view over an assembled drivetrain and its installed assists.
pub struct AssistedDrivetrain<'a, M: Drive, T: TrackingSystem, L, A> {
    drivetrain: &'a mut Drivetrain<M, T, L>,
    assists: &'a mut DriveAssists<A>,
}

impl<'a, M: Drive, T: TrackingSystem, L, A> AssistedDrivetrain<'a, M, T, L, A> {
    pub fn new(drivetrain: &'a mut Drivetrain<M, T, L>, assists: &'a mut DriveAssists<A>) -> Self {
        Self {
            drivetrain,
            assists,
        }
    }

    pub fn drivetrain(&self) -> &Drivetrain<M, T, L> {
        self.drivetrain
    }

    pub fn drivetrain_mut(&mut self) -> &mut Drivetrain<M, T, L> {
        self.drivetrain
    }
}

impl<M: Drive, T: AssistTracking, L, A: DriveAssist<M, T, L>> AssistedDrivetrain<'_, M, T, L, A> {
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
        let orientation = self
            .assists
            .items
            .needs_orientation()
            .then(|| self.drivetrain.tracking.assist_orientation())
            .flatten();
        let context = AssistContext {
            model: &self.drivetrain.model,
            tracking: &self.drivetrain.tracking,
            localization: &self.drivetrain.localization,
            orientation,
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

impl<M: Drive, T: AssistTracking, L, A: DriveAssist<M, T, L>> Drive
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

impl<M: LateralDrive, T: AssistTracking, L, A: DriveAssist<M, T, L>> LateralDrive
    for AssistedDrivetrain<'_, M, T, L, A>
{
}

impl<M: Drive, T: TrackingSystem, L, A> HasLocalization for AssistedDrivetrain<'_, M, T, L, A>
where
    Drivetrain<M, T, L>: HasLocalization<Localization = L>,
{
    type Localization = L;

    fn localization(&self) -> &L {
        self.drivetrain.localization()
    }

    fn refresh_localization(&mut self) {
        self.drivetrain.refresh_localization();
    }
}

impl<M: Drive, T: TrackingSystem, L, A> DriveEncoders for AssistedDrivetrain<'_, M, T, L, A>
where
    Drivetrain<M, T, L>: DriveEncoders,
{
    fn track_width(&self) -> Option<Length> {
        DriveEncoders::track_width(self.drivetrain)
    }

    fn tracking_positions(&mut self) -> [Option<Length>; 2] {
        self.drivetrain.tracking_positions()
    }
}

impl<M: Drive<Error = PortError> + MotorGroups, T: AssistTracking, L, A: DriveAssist<M, T, L>>
    AssistedDrivetrain<'_, M, T, L, A>
{
    /// Apply assists to a two-sided physical wheel velocity command.
    pub fn drive_wheel_speeds(&mut self, speeds: [Velocity; 2]) -> Result<(), WheelSpeedError> {
        let speeds_raw = speeds.map(|speed| speed.get::<meter_per_second>());
        let scale = speeds_raw[0].abs().max(speeds_raw[1].abs());
        if !speeds_raw.iter().all(|speed| speed.is_finite()) || scale == 0.0 {
            self.reset();
            return self.drivetrain.drive_wheel_speeds(speeds);
        }
        let (forward, turn) = tank_axes(speeds_raw.map(|speed| speed / scale));
        let (wheels, adjustment) = self.mix([forward, turn], true);
        self.drivetrain
            .drive_wheel_speeds(
                desaturate(wheels, 1.0)
                    .map(|speed| Velocity::new::<meter_per_second>(speed * scale)),
            )
            .and(adjustment.map_err(WheelSpeedError::Port))
    }
}

#[cfg(test)]
mod tests;
