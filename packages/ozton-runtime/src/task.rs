//! Results from background work owned by a competition component.

use std::{
    cell::RefCell,
    future::Future,
    pin::Pin,
    rc::Weak,
    task::{Context, Poll, Waker},
    time::{Duration, Instant},
};

use vexide::{
    competition::{self, CompetitionMode},
    task::{Task, spawn},
    time::sleep,
};

use crate::competition::Mechanism;

/// Owns one fallible background task. Check it from the owning component's tick
/// so errors reach the competition runtime through the component's result.
pub struct TaskSlot<E>(Option<Task<Result<(), E>>>);

impl<E> Default for TaskSlot<E> {
    fn default() -> Self {
        Self(None)
    }
}

impl<E> TaskSlot<E> {
    /// Includes a finished task whose result has yet to be collected.
    pub fn is_occupied(&self) -> bool {
        self.0.is_some()
    }

    /// Spawn fallible work when the slot is empty.
    pub fn start(&mut self, work: impl Future<Output = Result<(), E>> + 'static)
    where
        E: 'static,
    {
        self.0.get_or_insert_with(|| spawn(work));
    }

    /// Keep a shared mechanism ticking while a route borrows other hardware.
    /// The worker brakes on failure before delivering the error to its owner.
    pub fn start_mechanism(
        &mut self,
        mechanism: Weak<RefCell<impl Mechanism<E> + 'static>>,
        mode: CompetitionMode,
        period: Duration,
    ) where
        E: std::fmt::Display + 'static,
    {
        self.start(async move {
            let mut previous = Instant::now();
            loop {
                sleep(period).await;
                let Some(mechanism) = mechanism.upgrade() else {
                    return Ok(());
                };
                let mut mechanism = mechanism.borrow_mut();
                if competition::mode() != mode {
                    return mechanism.stop();
                }
                let now = Instant::now();
                let elapsed = now.saturating_duration_since(std::mem::replace(&mut previous, now));
                if let Err(error) = mechanism.update(mode, elapsed) {
                    if let Err(_stop) = mechanism.stop() {
                        crate::log_event!(
                            Error,
                            "mechanism",
                            "Could not stop outputs after background task failed ({_stop})"
                        );
                    }
                    return Err(error);
                }
            }
        });
    }

    /// Check without waiting. A completed result is delivered only once.
    pub fn check(&mut self) -> Result<(), E> {
        let Some(task) = self.0.as_mut().filter(|task| task.is_finished()) else {
            return Ok(());
        };
        let mut context = Context::from_waker(Waker::noop());
        match Pin::new(task).poll(&mut context) {
            Poll::Pending => Ok(()),
            Poll::Ready(result) => {
                self.0 = None;
                result
            }
        }
    }

    /// Cancel pending work and return any already-completed failure.
    pub fn cancel(&mut self) -> Result<(), E> {
        let result = self.check();
        self.0 = None;
        result
    }
}

#[cfg(all(test, not(target_os = "vexos")))]
mod tests {
    use vex_sdk_mock as _;

    use super::*;

    #[test]
    fn completed_failure_is_collected_once_and_cancel_clears_pending_work() {
        vexide::runtime::block_on(async {
            let mut slot = TaskSlot::default();
            slot.start(async { Err("motor fault") });
            assert!(slot.is_occupied());
            assert_eq!(slot.check(), Ok(()));
            spawn(async {}).await;
            assert_eq!(slot.check(), Err("motor fault"));
            assert!(!slot.is_occupied());
            assert_eq!(slot.check(), Ok(()));

            slot.start(std::future::pending());
            assert_eq!(slot.cancel(), Ok(()));
            assert!(!slot.is_occupied());
        });
    }
}
