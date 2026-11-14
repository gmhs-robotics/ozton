//! Bounded progress and encoder checks shared by mechanisms and motion executors.
use core::time::Duration;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProgressFault {
    InvalidFeedback,
    EncoderJump,
    NoProgress,
}

/// Positions use the caller's consistent units. A nonzero command must produce
/// signed physical travel before the deadline. Releasing the command rearms a jam;
/// invalid feedback and jumps require explicit acknowledgment with `reset`.
#[derive(Debug, Clone)]
pub struct ProgressGuard {
    minimum_progress: f64,
    maximum_speed: f64,
    timeout: Duration,
    previous: Option<f64>,
    anchor: Option<f64>,
    elapsed: Duration,
    direction: f64,
    fault: Option<ProgressFault>,
    velocity: f64,
}
impl ProgressGuard {
    pub fn new(minimum_progress: f64, maximum_speed: f64, timeout: Duration) -> Self {
        Self {
            minimum_progress,
            maximum_speed,
            timeout,
            previous: None,
            anchor: None,
            elapsed: Duration::ZERO,
            direction: 0.0,
            fault: None,
            velocity: 0.0,
        }
    }
    pub fn reset(&mut self) {
        self.previous = None;
        self.anchor = None;
        self.elapsed = Duration::ZERO;
        self.direction = 0.0;
        self.fault = None;
        self.velocity = 0.0;
    }
    pub fn velocity(&self) -> f64 {
        self.velocity
    }
    pub fn fault(&self) -> Option<ProgressFault> {
        self.fault
    }
    pub fn update(
        &mut self,
        position: f64,
        command: f64,
        dt: Duration,
    ) -> Result<(), ProgressFault> {
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
        }
        if let Some(last) = self.previous {
            let delta = position - last;
            self.velocity = delta / dt.as_secs_f64();
            if delta.abs() > self.maximum_speed * dt.as_secs_f64() + self.minimum_progress {
                self.fault = Some(ProgressFault::EncoderJump);
            }
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
    use super::*;
    #[test]
    fn jam_trips_release_rearms_and_jump_needs_acknowledgment() {
        let mut guard = ProgressGuard::new(0.01, 2.0, Duration::from_millis(100));
        let tick = Duration::from_millis(10);
        for _ in 0..9 {
            assert!(guard.update(0.0, 1.0, tick).is_ok());
        }
        assert_eq!(guard.update(0.0, 1.0, tick), Err(ProgressFault::NoProgress));
        assert!(guard.update(0.0, 0.0, tick).is_ok());
        assert!(guard.update(0.01, 1.0, tick).is_ok());
        assert_eq!(
            guard.update(1.0, 1.0, tick),
            Err(ProgressFault::EncoderJump)
        );
        assert!(guard.update(1.0, 0.0, tick).is_err());
        guard.reset();
        assert!(guard.update(1.0, 0.0, tick).is_ok());
    }
    #[test]
    fn wrong_direction_does_not_count_as_progress() {
        let mut guard = ProgressGuard::new(0.01, 2.0, Duration::from_millis(100));
        for i in 0..10 {
            let _ = guard.update(-0.01 * i as f64, 1.0, Duration::from_millis(10));
        }
        assert_eq!(guard.fault(), Some(ProgressFault::NoProgress));
    }
}
