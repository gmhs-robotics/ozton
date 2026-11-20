//! Measured and deadline-based control ticks.

use std::time::{Duration, Instant};

use vexide::time::sleep_until;

pub const CONTROL_PERIOD: Duration = Duration::from_millis(10);

pub struct CommandClock {
    previous: Instant,
    period: Duration,
}

impl CommandClock {
    pub fn new(period: Duration) -> Option<Self> {
        let previous = Instant::now();
        if period.is_zero() {
            return None;
        }
        previous.checked_add(period)?;
        Some(Self { previous, period })
    }

    /// Includes work since the previous tick and avoids catch-up bursts after a stall.
    pub async fn tick(&mut self) -> Duration {
        if let Some(deadline) = self.previous.checked_add(self.period) {
            sleep_until(deadline).await;
        }
        let now = Instant::now();
        let elapsed = now.saturating_duration_since(self.previous);
        self.previous = now;
        elapsed
    }
}
