//! Run a drivetrain's localization at the robot's control period.

use ozton_localization::{Localization, field::Boundary};
use ozton_tracking::{Tracking, TrackingSources, TrackingSystem};
use vexide::competition::CompetitionMode;
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

/// An estimator that can consume samples from the given tracking configuration.
pub trait LocalizationInput<T> {
    fn sample(&mut self, tracking: &mut T);
    fn pause(&mut self);

    #[cfg(feature = "logging")]
    #[doc(hidden)]
    fn report(&self, _tracking: &T, _frame: crate::diagnostics::Frame) {}
}

impl<S: TrackingSources, B: Boundary> LocalizationInput<Tracking<S>> for Localization<B>
where
    Tracking<S>: TrackingSystem,
{
    fn sample(&mut self, tracking: &mut Tracking<S>) {
        const {
            assert!(
                S::DIFFERENTIAL_COUNT == 0 || <Tracking<S> as TrackingSystem>::HAS_SELECTED_DRIVE,
                "install differential encoders with Sources::with_drive before running localization"
            );
        }
        let width = tracking.selected_drive().map(|source| source.track_width());
        let span = crate::profile::Span::new(crate::profile::Stage::Tracking);
        let sample = tracking.sample(self.occluded());
        drop(span);
        let _span = crate::profile::Span::new(crate::profile::Stage::Localization);
        let _ = self.update(sample, width);
    }

    #[cfg(feature = "logging")]
    fn report(&self, tracking: &Tracking<S>, frame: crate::diagnostics::Frame) {
        use crate::diagnostics::ComponentReport;
        ComponentReport::report(self, frame);
        tracking.report(frame);
    }

    fn pause(&mut self) {
        self.cancel_distance_reset();
        Localization::pause(self);
    }
}

#[cfg(all(test, not(target_os = "vexos")))]
mod tests {
    use std::convert::Infallible;

    use ozton_drivetrain::Drive;
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
    fn autonomous_only_gates_sampling_by_mode() {
        assert!(LocalizationSchedule::Autonomous.runs_in(CompetitionMode::Autonomous));
        assert!(!LocalizationSchedule::Autonomous.runs_in(CompetitionMode::Disabled));
        assert!(LocalizationSchedule::Always.runs_in(CompetitionMode::Disabled));
    }
}
