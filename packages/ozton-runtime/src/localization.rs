//! Run a drivetrain's localization at the robot's control period.

use std::time::{Duration, Instant};

use ozton_drivetrain::{Drive, Drivetrain};
use ozton_localization::differential::DifferentialLocalization;
use ozton_tracking::{Tracking, TrackingSources, TrackingSystem};
use vexide::{
    competition::{self, CompetitionMode},
    task::{Task, spawn},
};

use crate::clock::sleep;
mod worker;
pub use worker::LocalizationWorker;

/// Modes in which the runtime should sample tracking and update the estimate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LocalizationSchedule {
    /// Update during disabled, autonomous, and driver control.
    Always,
    /// Update only in autonomous. Reestablish sensor baselines after a pause.
    Autonomous,
}

impl LocalizationSchedule {
    const fn runs_in(self, mode: CompetitionMode) -> bool {
        matches!(self, Self::Always) || matches!(mode, CompetitionMode::Autonomous)
    }
}

/// Connect a drivetrain's sources and estimator to the runtime.
pub trait ScheduledLocalization {
    fn start_localization(
        &self,
        schedule: LocalizationSchedule,
        mode: CompetitionMode,
        period: Duration,
    ) -> Option<Task<()>>;
}

/// An estimator that can consume samples from the given tracking configuration.
pub trait LocalizationInput<T> {
    fn sample(&mut self, tracking: &mut T);
    fn pause(&mut self);
}

impl<S: TrackingSources> LocalizationInput<Tracking<S>> for DifferentialLocalization {
    fn sample(&mut self, tracking: &mut Tracking<S>) {
        let width = tracking.drive_wheels().map(|source| source.track_width());
        let sample = tracking.sample(self.occluded());
        let _ = self.update(sample, width);
    }

    fn pause(&mut self) {
        self.cancel_distance_reset();
        DifferentialLocalization::pause(self);
    }
}

impl<M: Drive, T: 'static, L: LocalizationInput<T> + 'static> ScheduledLocalization
    for Drivetrain<M, T, L>
{
    fn start_localization(
        &self,
        schedule: LocalizationSchedule,
        mode: CompetitionMode,
        period: Duration,
    ) -> Option<Task<()>> {
        let state = self.localization_state();
        if !schedule.runs_in(mode) {
            state.upgrade()?.try_borrow_mut().ok()?.1.pause();
            return None;
        }
        Some(spawn(async move {
            while competition::mode() == mode {
                let started = Instant::now();
                {
                    let Some(state) = state.upgrade() else { break };
                    if let Ok(mut state) = state.try_borrow_mut() {
                        let (tracking, localization) = &mut *state;
                        localization.sample(tracking);
                    }
                }
                sleep(
                    period
                        .saturating_sub(started.elapsed())
                        .max(Duration::from_millis(1)),
                )
                .await;
            }
        }))
    }
}

type Start<D> = fn(&D, LocalizationSchedule, CompetitionMode, Duration) -> Option<Task<()>>;

pub(crate) struct Schedule<D> {
    modes: LocalizationSchedule,
    start: Start<D>,
}

impl<D> Schedule<D> {
    pub(crate) fn new(modes: LocalizationSchedule) -> Self
    where
        D: ScheduledLocalization,
    {
        Self {
            modes,
            start: D::start_localization,
        }
    }

    pub(crate) fn start(
        &self,
        drive: &D,
        mode: CompetitionMode,
        period: Duration,
    ) -> Option<Task<()>> {
        (self.start)(drive, self.modes, mode, period)
    }
}

#[cfg(all(test, not(target_os = "vexos")))]
mod tests {
    use std::convert::Infallible;

    use ozton_math::geometry::DVec2;
    use vex_sdk_mock as _;

    use super::*;

    pub(super) struct TestDrive;

    impl Drive for TestDrive {
        type Error = Infallible;

        fn drive_vector(&mut self, _: DVec2, _: f64) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[derive(Default)]
    pub(super) struct Counts {
        pub(super) samples: usize,
        pub(super) pauses: usize,
    }

    impl LocalizationInput<()> for Counts {
        fn sample(&mut self, _: &mut ()) {
            self.samples += 1;
        }

        fn pause(&mut self) {
            self.pauses += 1;
        }
    }

    #[test]
    fn autonomous_only_skips_driver_updates_and_always_samples_until_cancelled() {
        assert!(LocalizationSchedule::Autonomous.runs_in(CompetitionMode::Autonomous));
        assert!(!LocalizationSchedule::Autonomous.runs_in(CompetitionMode::Disabled));
        assert!(LocalizationSchedule::Always.runs_in(CompetitionMode::Disabled));

        vexide::runtime::block_on(async {
            let drive = Drivetrain::new(TestDrive, (), Counts::default());
            let mode = CompetitionMode::Driver; // Mock competition status is zero.
            let period = Duration::from_millis(1);
            assert_eq!(competition::mode(), mode);
            assert!(
                drive
                    .start_localization(LocalizationSchedule::Autonomous, mode, period)
                    .is_none()
            );
            assert_eq!(drive.localization().pauses, 1);
            assert_eq!(drive.localization().samples, 0);

            let task = drive
                .start_localization(LocalizationSchedule::Always, mode, period)
                .unwrap();
            let started = Instant::now();
            while drive.localization().samples < 2 {
                assert!(started.elapsed() < Duration::from_secs(1));
                sleep(period).await;
            }
            let samples = drive.localization().samples;
            drop(task);
            sleep(period).await;
            assert_eq!(drive.localization().samples, samples);
        });
    }
}
