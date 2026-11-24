//! One-shot nonblocking mechanism actions tied to geometric path progress.

use std::time::Duration;

use ozton_math::{path::Path, uom::si::f64::Length};
use ozton_runtime::mechanism::MechanismResult;

#[derive(Clone, Copy)]
enum Trigger {
    Distance(Length),
    Remaining(Duration),
    Arrival,
}

type Action<'a> = Box<dyn FnOnce() -> MechanismResult + 'a>;

struct Event<'a> {
    trigger: Trigger,
    action: Option<Action<'a>>,
}

/// One-shot actions for a single trajectory. Callbacks start mechanisms
/// serviced independently of the drive loop; they must not block.
#[derive(Default)]
pub struct PathActions<'a> {
    events: Vec<Event<'a>>,
}

impl<'a> PathActions<'a> {
    pub const fn new() -> Self {
        Self { events: Vec::new() }
    }

    #[must_use]
    pub fn on_start(self, action: impl FnOnce() -> MechanismResult + 'a) -> Self {
        self.at_distance(ozton_math::qty!(0 m), action)
    }

    #[must_use]
    pub fn at_distance(
        mut self,
        distance: Length,
        action: impl FnOnce() -> MechanismResult + 'a,
    ) -> Self {
        self.events.push(Event {
            trigger: Trigger::Distance(distance),
            action: Some(Box::new(action)),
        });
        self
    }

    /// Start when the nominal profile time remaining reaches a measured
    /// mechanism lead time. This does not guarantee physical readiness.
    #[must_use]
    pub fn before_end(
        mut self,
        lead: Duration,
        action: impl FnOnce() -> MechanismResult + 'a,
    ) -> Self {
        self.events.push(Event {
            trigger: Trigger::Remaining(lead),
            action: Some(Box::new(action)),
        });
        self
    }

    #[must_use]
    pub fn on_arrival(mut self, action: impl FnOnce() -> MechanismResult + 'a) -> Self {
        self.events.push(Event {
            trigger: Trigger::Arrival,
            action: Some(Box::new(action)),
        });
        self
    }

    pub(super) fn valid(&self, path: &Path) -> bool {
        self.events.iter().all(|event| match event.trigger {
            Trigger::Distance(distance) => {
                distance.value.is_finite() && (0.0..=path.end().distance).contains(&distance.value)
            }
            _ => true,
        })
    }

    pub(super) fn advance(&mut self, distance: f64, remaining: f64, arrived: bool) -> MechanismResult {
        for event in &mut self.events {
            let ready = match event.trigger {
                Trigger::Distance(target) => arrived || distance >= target.value,
                Trigger::Remaining(lead) => arrived || remaining <= lead.as_secs_f64(),
                Trigger::Arrival => arrived,
            };
            if ready && let Some(action) = event.action.take() {
                action()?;
            }
        }
        Ok(())
    }
}
