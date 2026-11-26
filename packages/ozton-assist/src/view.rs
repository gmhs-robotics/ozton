//! Borrow any drive for assisted commands while keeping controller state across ticks.
use std::time::{Duration, Instant};

use ozton_drivetrain::{Drive, LateralDrive, model::tank_axes};
use ozton_math::{
    desaturate,
    geometry::DVec2,
    uom::si::{f64::Velocity, velocity::meter_per_second},
};

use super::{AssistContext, DriveAssist};

mod capabilities;
#[cfg(test)]
mod tests;

const MAX_CONTROL_GAP: Duration = Duration::from_millis(100);

/// Assist configuration and timing state, retained across borrowed drive views.
pub struct DriveAssists<A> {
    items: A,
    last_tick: Option<(Instant, bool)>,
}

impl<A> DriveAssists<A> {
    pub const fn new(items: A) -> Self {
        Self {
            items,
            last_tick: None,
        }
    }

    /// Borrow a drive for commands with these assists applied.
    pub fn apply_to<'a, D: Drive + ?Sized>(
        &'a mut self,
        drive: &'a mut D,
    ) -> AssistedDrive<'a, D, A>
    where
        A: DriveAssist<D>,
    {
        AssistedDrive {
            drive,
            assists: self,
        }
    }
}

/// Drive view with statically dispatched command transforms.
pub struct AssistedDrive<'a, D: ?Sized, A> {
    drive: &'a mut D,
    assists: &'a mut DriveAssists<A>,
}

impl<D: Drive + ?Sized, A: DriveAssist<D>> AssistedDrive<'_, D, A> {
    fn mix(
        &mut self,
        mut axes: [f64; 2],
        wheel_velocity: bool,
    ) -> ([f64; 2], Result<(), D::Error>) {
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
        let context = AssistContext {
            drive: self.drive,
            elapsed,
            wheel_velocity,
        };
        self.assists.items.apply(&mut axes, &context);
        let [forward, turn] = axes;
        let mut wheels = [forward + turn, forward - turn];
        let result = self.assists.items.wheels(&context, &mut wheels);
        (wheels, result)
    }

    fn mix_velocity(&mut self, speeds: [Velocity; 2]) -> ([Velocity; 2], Result<(), D::Error>) {
        let [left, right] = speeds.map(|speed| speed.value);
        let scale = left.abs().max(right.abs());
        if !left.is_finite() || !right.is_finite() || scale == 0.0 {
            self.reset();
            return (speeds, Ok(()));
        }

        let (forward, turn) = tank_axes([left / scale, right / scale]);
        let (wheels, result) = self.mix([forward, turn], true);
        let speeds =
            desaturate(wheels, 1.0).map(|power| Velocity::new::<meter_per_second>(power * scale));
        (speeds, result)
    }

    fn reset(&mut self) {
        self.assists.last_tick = None;
        self.assists.items.reset();
    }
}

impl<D: Drive + ?Sized, A: DriveAssist<D>> Drive for AssistedDrive<'_, D, A> {
    type Error = D::Error;

    fn drive_vector(&mut self, mut vector: DVec2, turn: f64) -> Result<(), Self::Error> {
        if !vector.is_finite() || !turn.is_finite() {
            self.reset();
            return self.drive.drive_vector(DVec2::ZERO, 0.0);
        }
        let (wheels, adjustment) = self.mix([vector.y, turn], false);
        let (forward, turn) = tank_axes(wheels);
        vector.y = forward;
        self.drive.drive_vector(vector, turn).and(adjustment)
    }

    fn stop_now(&mut self) -> Result<(), Self::Error> {
        self.reset();
        self.drive.stop_now()
    }
}

impl<D: LateralDrive + ?Sized, A: DriveAssist<D>> LateralDrive for AssistedDrive<'_, D, A> {}
