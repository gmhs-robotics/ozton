//! Competition-lifetime tracking and estimation task.

use std::{
    cell::Cell,
    rc::Rc,
    time::{Duration, Instant},
};

use ozton_drivetrain::{Drive, Drivetrain};
use vexide::{
    competition::{self, CompetitionMode},
    task::{Task, spawn},
};

use super::{LocalizationInput, LocalizationSchedule};
use crate::clock::sleep;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Sampling {
    Running,
    PausePending,
    Paused,
}

/// Keeps the sampling task alive and synchronizes its gate before mode callbacks.
pub struct LocalizationWorker {
    _task: Task<()>,
    #[cfg(feature = "logging")]
    _diagnostics: Rc<dyn Fn(crate::diagnostics::Frame)>,
    synchronize: Rc<dyn Fn(CompetitionMode, bool) -> bool>,
}

impl LocalizationWorker {
    pub fn start<M, T, L>(
        drive: &Drivetrain<M, T, L>,
        schedule: LocalizationSchedule,
        period: Duration,
    ) -> Self
    where
        M: Drive,
        T: 'static,
        L: LocalizationInput<T> + 'static,
    {
        let state = drive.localization_state();
        #[cfg(feature = "logging")]
        let diagnostics = {
            let state = state.clone();
            let stamp = drive.diagnostic_stamp();
            crate::diagnostics::register(move |frame| {
                let Some(state) = state.upgrade() else { return };
                let Ok(state) = state.try_borrow() else { return };
                if crate::diagnostics::claim_drive_sample(&stamp, frame.time) {
                    state.1.report(&state.0, frame);
                }
            })
        };
        let sampling = Cell::new(Sampling::Running);
        let previous_activation = Cell::new(None);
        let synchronize = Rc::new(move |mode, sample| {
            let enabled = schedule.runs_in(mode);
            if !sample {
                let activation = crate::competition::activation::current();
                let previous = previous_activation.replace(Some((mode, activation)));
                if schedule == LocalizationSchedule::Autonomous
                    && previous.is_some_and(|(old_mode, old_activation)| {
                        old_mode == CompetitionMode::Autonomous
                            && mode == CompetitionMode::Autonomous
                            && old_activation != activation
                    })
                {
                    // A new autonomous callback can follow a very brief
                    // intervening mode that the sampling task never observed.
                    sampling.set(Sampling::PausePending);
                }
            }
            if !enabled && sampling.get() == Sampling::Running {
                sampling.set(Sampling::PausePending);
            }

            let Some(state) = state.upgrade() else {
                return false;
            };
            if !enabled && sampling.get() == Sampling::Paused {
                return true;
            }
            if let Ok(mut state) = state.try_borrow_mut() {
                let (tracking, localization) = &mut *state;
                if sampling.get() == Sampling::PausePending {
                    localization.pause();
                    sampling.set(Sampling::Paused);
                }
                if enabled {
                    sampling.set(Sampling::Running);
                    if sample {
                        localization.sample(tracking);
                    }
                }
            }
            true
        });
        let check = Rc::clone(&synchronize);
        let task = spawn(async move {
            loop {
                let started = Instant::now();
                if !check(competition::mode(), true) {
                    break;
                }
                sleep(
                    period
                        .saturating_sub(started.elapsed())
                        .max(Duration::from_millis(1)),
                )
                .await;
            }
        });
        Self {
            _task: task,
            #[cfg(feature = "logging")]
            _diagnostics: diagnostics,
            synchronize,
        }
    }

    pub(crate) fn mode_changed(&self, mode: CompetitionMode) {
        (self.synchronize)(mode, false);
    }
}

#[cfg(all(test, not(target_os = "vexos")))]
mod tests {
    use vex_sdk_mock as _;

    use super::*;
    use crate::localization::tests::{Counts, TestDrive};

    #[test]
    fn rapid_autonomous_reentry_rebaselines_before_new_route() {
        vexide::runtime::block_on(async {
            let drive = Drivetrain::new(TestDrive, (), Counts::default());
            let worker = LocalizationWorker::start(
                &drive,
                LocalizationSchedule::Autonomous,
                Duration::from_millis(1),
            );
            worker.mode_changed(CompetitionMode::Autonomous);
            assert_eq!(drive.localization().pauses, 0);
            crate::competition::activation::advance();
            worker.mode_changed(CompetitionMode::Autonomous);
            assert_eq!(drive.localization().pauses, 1);
            worker.mode_changed(CompetitionMode::Autonomous);
            assert_eq!(drive.localization().pauses, 1);
        });
    }

    #[test]
    fn pending_pause_is_applied_before_the_next_route_can_seed() {
        vexide::runtime::block_on(async {
            let mut drive = Drivetrain::new(TestDrive, (), Counts::default());
            let period = Duration::from_millis(1);
            let worker =
                LocalizationWorker::start(&drive, LocalizationSchedule::Autonomous, period);

            {
                let read = drive.localization();
                worker.mode_changed(CompetitionMode::Disabled);
                assert_eq!(read.pauses, 0);
            }

            worker.mode_changed(CompetitionMode::Autonomous);
            assert_eq!(drive.localization().pauses, 1);
            drive.localization_mut().samples = 10; // Represent a newly seeded estimate.
            (worker.synchronize)(CompetitionMode::Autonomous, true);
            assert_eq!(drive.localization().samples, 11);
            assert_eq!(drive.localization().pauses, 1);

            // The SDK mock stays in Driver. The same task observes its gated mode.
            sleep(Duration::from_millis(5)).await;
            assert_eq!(drive.localization().pauses, 2);
            assert_eq!(drive.localization().samples, 11);
            sleep(Duration::from_millis(5)).await;
            assert_eq!(drive.localization().pauses, 2);
        });
    }
}
