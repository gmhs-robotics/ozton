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

use crate::competition::{Mechanism, activation, while_in_mode};

/// Owns one fallible background task. Check it from the owning component's tick
/// so errors reach the competition runtime through the component's result.
pub struct TaskSlot<E>(Option<ScopedTask<E>>);

struct ScopedTask<E> {
    task: Task<Result<(), E>>,
    token: activation::Token,
}

impl<E> ScopedTask<E> {
    fn active(&self) -> bool {
        self.token.active()
    }
}

impl<E> Default for TaskSlot<E> {
    fn default() -> Self {
        Self(None)
    }
}

impl<E> TaskSlot<E> {
    /// Includes a finished task whose result has yet to be collected.
    pub fn is_occupied(&self) -> bool {
        self.0.as_ref().is_some_and(ScopedTask::active)
    }

    /// Spawn fallible work when the slot is empty.
    pub fn start(&mut self, work: impl Future<Output = Result<(), E>> + 'static)
    where
        E: 'static,
    {
        if self.0.as_ref().is_some_and(|task| !task.active()) {
            self.0 = None;
        }
        self.0.get_or_insert_with(|| {
            let mode = competition::mode();
            let token = activation::Token::capture(mode);
            let task = spawn(async move {
                // Check even before the first poll: a task may have been
                // queued in the preceding activation of the same mode.
                if !token.active() {
                    return Ok(());
                }
                while_in_mode(work, mode, crate::clock::CONTROL_PERIOD)
                    .await
                    .unwrap_or(Ok(()))
            });
            ScopedTask { task, token }
        });
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
        if competition::mode() != mode {
            return;
        }
        self.start(async move {
            let mut previous = Instant::now();
            loop {
                sleep(period.max(Duration::from_millis(1))).await;
                if competition::mode() != mode {
                    return Ok(());
                }
                let Some(mechanism) = mechanism.upgrade() else {
                    return Ok(());
                };
                let Ok(mut mechanism) = mechanism.try_borrow_mut() else {
                    continue;
                };
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
        if self.0.as_ref().is_some_and(|task| !task.active()) {
            self.0 = None;
            return Ok(());
        }
        let Some(task) = self
            .0
            .as_mut()
            .map(|task| &mut task.task)
            .filter(|task| task.is_finished())
        else {
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
    fn stale_tasks_cannot_write_after_reentering_the_same_mode() {
        use std::{cell::Cell, rc::Rc};

        vexide::runtime::block_on(async {
            let writes = Rc::new(Cell::new(0));
            let mut slot = TaskSlot::<&'static str>::default();
            let first = activation::Token::capture(competition::mode());
            slot.start({
                let writes = Rc::clone(&writes);
                async move {
                    sleep(Duration::from_millis(10)).await;
                    writes.set(writes.get() + 1);
                    Err("stale fault")
                }
            });
            sleep(Duration::from_millis(1)).await;
            crate::competition::activation::advance();
            assert!(!first.active());
            assert!(!slot.is_occupied());
            slot.start({
                let writes = Rc::clone(&writes);
                async move {
                    writes.set(writes.get() + 10);
                    Ok(())
                }
            });
            sleep(Duration::from_millis(20)).await;
            assert_eq!(writes.get(), 10);
            assert_eq!(slot.check(), Ok(()));
        });
    }

    #[test]
    #[allow(
        clippy::await_holding_refcell_ref,
        reason = "The competing borrow must span worker ticks to exercise contention handling"
    )]
    fn mechanism_worker_skips_conflicting_borrows() {
        use std::rc::Rc;

        #[derive(Default)]
        struct Counter(usize);

        impl Mechanism<&'static str> for Counter {
            fn update(&mut self, _: CompetitionMode, _: Duration) -> Result<(), &'static str> {
                self.0 += 1;
                Ok(())
            }
        }

        vexide::runtime::block_on(async {
            let mechanism = Rc::new(RefCell::new(Counter::default()));
            let mut slot = TaskSlot::default();
            slot.start_mechanism(
                Rc::downgrade(&mechanism),
                competition::mode(),
                Duration::from_millis(1),
            );
            {
                let held = mechanism.borrow_mut();
                sleep(Duration::from_millis(5)).await;
                assert_eq!(held.0, 0);
            }
            sleep(Duration::from_millis(5)).await;
            assert!(mechanism.borrow().0 > 0);
            assert_eq!(slot.cancel(), Ok(()));
        });
    }

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
