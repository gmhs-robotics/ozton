//! Bounded progress and encoder checks shared by mechanisms and motion executors.
use core::{marker::PhantomData, time::Duration};

use crate::uom::ConstZero;

use super::axis::{AxisDimension, Position, Rotary, Velocity};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProgressFault {
    InvalidFeedback,
    EncoderJump,
    NoProgress,
}

/// Positions and speeds follow the axis dimension. A nonzero command must produce
/// signed physical travel before the deadline. Releasing the command rearms a jam;
/// invalid feedback and jumps require explicit acknowledgment with `reset`.
#[derive(Debug, Clone)]
pub struct ProgressGuard<D: AxisDimension = Rotary> {
    dimension: PhantomData<D>,
    minimum_progress: f64,
    maximum_speed: f64,
    timeout: Duration,
    previous: Option<f64>,
    anchor: Option<f64>,
    elapsed: Duration,
    direction: f64,
    fault: Option<ProgressFault>,
    velocity: Velocity<D>,
}
impl<D: AxisDimension> ProgressGuard<D> {
    pub fn new(
        minimum_progress: Position<D>,
        maximum_speed: Velocity<D>,
        timeout: Duration,
    ) -> Self {
        Self {
            dimension: PhantomData,
            minimum_progress: minimum_progress.value,
            maximum_speed: maximum_speed.value,
            timeout,
            previous: None,
            anchor: None,
            elapsed: Duration::ZERO,
            direction: 0.0,
            fault: None,
            velocity: Velocity::<D>::ZERO,
        }
    }
    pub fn reset(&mut self) {
        self.previous = None;
        self.anchor = None;
        self.elapsed = Duration::ZERO;
        self.direction = 0.0;
        self.fault = None;
        self.velocity = Velocity::<D>::ZERO;
    }
    pub fn velocity(&self) -> Velocity<D> {
        self.velocity
    }
    pub fn fault(&self) -> Option<ProgressFault> {
        self.fault
    }
    pub fn update(
        &mut self,
        position: Position<D>,
        command: f64,
        dt: Duration,
    ) -> Result<(), ProgressFault> {
        let position = position.value;
        if let Some(fault @ (ProgressFault::InvalidFeedback | ProgressFault::EncoderJump)) =
            self.fault
        {
            return Err(fault);
        }
        if !position.is_finite()
            || !command.is_finite()
            || dt.is_zero()
            || !self.minimum_progress.is_finite()
            || self.minimum_progress <= 0.0
            || !self.maximum_speed.is_finite()
            || self.maximum_speed <= 0.0
            || self.timeout.is_zero()
        {
            self.fault = Some(ProgressFault::InvalidFeedback);
            return Err(ProgressFault::InvalidFeedback);
        }
        if let Some(last) = self.previous {
            let delta = position - last;
            let seconds = dt.as_secs_f64();
            let velocity = delta / seconds;
            if !velocity.is_finite()
                || delta.abs() > self.maximum_speed * seconds + self.minimum_progress
            {
                self.fault = Some(ProgressFault::EncoderJump);
                return Err(ProgressFault::EncoderJump);
            }
            self.velocity.value = velocity;
        }
        self.previous = Some(position);
        if command.abs() < 0.05 {
            self.anchor = None;
            self.elapsed = Duration::ZERO;
            self.direction = 0.0;
            if self.fault == Some(ProgressFault::NoProgress) {
                self.fault = None;
            }
        }
        if let Some(fault) = self.fault {
            return Err(fault);
        }
        if command.abs() < 0.05 {
            return Ok(());
        }
        let direction = command.signum();
        if self.direction != direction {
            self.anchor = None;
            self.elapsed = Duration::ZERO;
        }
        self.direction = direction;
        let anchor = *self.anchor.get_or_insert(position);
        if (position - anchor) * direction >= self.minimum_progress {
            self.anchor = Some(position);
            self.elapsed = Duration::ZERO;
        } else {
            self.elapsed = self.elapsed.saturating_add(dt);
            if self.elapsed >= self.timeout {
                self.fault = Some(ProgressFault::NoProgress);
                return Err(ProgressFault::NoProgress);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::uom::si::{angular_velocity::radian_per_second, f64::AngularVelocity};

    use super::*;
    #[test]
    fn jam_trips_release_rearms_and_jump_needs_acknowledgment() {
        let mut guard = ProgressGuard::<Rotary>::new(
            crate::qty!(0.01 rad),
            AngularVelocity::new::<radian_per_second>(2.0),
            Duration::from_millis(100),
        );
        let tick = Duration::from_millis(10);
        for _ in 0..9 {
            assert!(guard.update(crate::qty!(0.0 rad), 1.0, tick).is_ok());
        }
        assert_eq!(
            guard.update(crate::qty!(0.0 rad), 1.0, tick),
            Err(ProgressFault::NoProgress)
        );
        assert!(guard.update(crate::qty!(0.0 rad), 0.0, tick).is_ok());
        assert!(guard.update(crate::qty!(0.01 rad), 1.0, tick).is_ok());
        assert_eq!(
            guard.update(crate::qty!(1.0 rad), 1.0, tick),
            Err(ProgressFault::EncoderJump)
        );
        assert!(guard.update(crate::qty!(1.0 rad), 0.0, tick).is_err());
        guard.reset();
        assert!(guard.update(crate::qty!(1.0 rad), 0.0, tick).is_ok());
    }
    #[test]
    fn wrong_direction_does_not_count_as_progress() {
        let mut guard = ProgressGuard::<Rotary>::new(
            crate::qty!(0.01 rad),
            AngularVelocity::new::<radian_per_second>(2.0),
            Duration::from_millis(100),
        );
        for i in 0..10 {
            let _ = guard.update(
                crate::qty!((-0.01 * f64::from(i)) rad),
                1.0,
                Duration::from_millis(10),
            );
        }
        assert_eq!(guard.fault(), Some(ProgressFault::NoProgress));
    }
}
