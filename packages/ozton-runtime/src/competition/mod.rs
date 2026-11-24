//! Programmed route selection and a budgeted competition loop for custom robots.

use std::time::{Duration, Instant};

use ozton_ui::RouteSelector;
pub use ozton_ui::Ui;
pub use vexide::competition::Compete;
use vexide::{competition::CompetitionMode, controller::Controller};

use crate::{
    Robot,
    clock::CONTROL_PERIOD,
    diagnostics::{ComponentReport, Report, Sample, Sampler, Telemetry},
};

mod dispatch;
mod drive;
pub use drive::ProgramDrive;
mod policy;
mod routes;
mod runtime;
mod selector_task;
mod session;

pub use policy::{CompetitionLayout, Mechanism, Program};
pub use routes::*;
pub use runtime::{OztonCompete, Runtime};
pub(crate) use session::compete;
use session::while_in_mode;

/// Timing for the current control tick, including time spent in application hooks.
#[derive(Clone, Copy)]
pub struct Tick {
    started: Instant,
    pub elapsed: Duration,
}

impl Tick {
    pub fn start(elapsed: Duration) -> Self {
        Self {
            started: Instant::now(),
            elapsed,
        }
    }

    pub fn work(self) -> Duration {
        self.started.elapsed()
    }
}

/// Opt-in competition hooks for an application with its own robot and controls.
/// Implementations own device errors and drive/motor policy. The loop owns mode
/// transitions, route selection, disable handling and the periodic tick.
#[allow(async_fn_in_trait)]
pub trait CompetitionRobot: Sized {
    type Error: std::fmt::Display;
    type Route: Routes<Self, Self::Error>;
    const CONTROL_PERIOD: Duration = CONTROL_PERIOD;

    fn menu(&mut self) -> &mut dyn RouteSelector<Option<Self::Route>>;
    fn stop(&mut self) -> Result<(), Self::Error>;
    async fn driver_tick(&mut self, tick: Tick);
    fn set_autonomous_start(&mut self, _started: Option<Instant>) {}

    fn selected_autonomous(&mut self) -> Option<Self::Route> {
        self.menu()
            .action()
            .filter(|route| route.allows_autonomous())
    }

    /// Run the driver session. The builder defaults to dispatching a callback each tick.
    async fn driver_mode(&mut self) -> Result<(), Self::Error> {
        session::CompetitionSession(self)
            .ticks(CompetitionMode::Driver)
            .await;
        Ok(())
    }

    fn driver_failed(&mut self, error: Self::Error) {
        crate::log_event!(Error, "driver", "Control fault ({error})");
        self.menu().status(&error.to_string());
    }

    async fn idle_mode(&mut self, mode: CompetitionMode) {
        session::CompetitionSession(self).ticks(mode).await;
    }

    fn mode_changed(&mut self, _mode: CompetitionMode) {}
    fn start_localization(&self, _mode: CompetitionMode) -> Option<vexide::task::Task<()>> {
        None
    }
    fn after_tick(&mut self, _mode: CompetitionMode, _tick: Tick) {}
    fn route_failed(&mut self, route: Self::Route, error: Self::Error) {
        crate::log_event!(
            Error,
            "autonomous",
            "Route {:?} stopped ({error})",
            route.label()
        );
        self.menu().status("Autonomous stopped");
    }
}

/// Brain selection and status for an assembled robot.
pub struct Autonomous<R: Copy + 'static, S = ()> {
    menu: Box<dyn RouteSelector<Option<R>>>,
    sampler: S,
    started: Option<Instant>,
    last_driver_fault: Option<String>,
    driver: Option<R>,
}

impl<R: Copy + 'static, S> Autonomous<R, S> {
    pub fn started_at(&self) -> Option<Instant> {
        self.started
    }

    /// Prevent further motion commands after a terminal failure in this route.
    pub fn cancel_motion(&mut self) {
        self.started = None;
    }

    pub fn status(&mut self, message: &str) {
        self.menu.status(message);
    }

    fn report_driver_fault(&mut self, fault: Option<String>) {
        if self.last_driver_fault == fault {
            return;
        }
        if let Some(message) = &fault {
            crate::log_event!(Error, "driver", "Control fault ({message})");
            self.menu.status(message);
        } else {
            crate::log_event!(Info, "driver", "Control recovered");
            self.menu.status(DRIVER_STATUS);
        }
        self.last_driver_fault = fault;
    }

    pub(crate) fn with_diagnostics<L>(self, interval: Duration) -> Autonomous<R, Sampler<L>> {
        Autonomous {
            menu: self.menu,
            sampler: Sampler::new(interval),
            started: self.started,
            last_driver_fault: self.last_driver_fault,
            driver: self.driver,
        }
    }
}

impl<R: RouteSet, S> Autonomous<R, S> {
    pub(crate) fn set_driver(&mut self, driver: R) {
        self.driver = Some(driver);
    }

    pub fn selected_label(&self) -> Option<&'static str> {
        self.menu.action().map(RouteSet::label)
    }
}

impl<R: RouteSet, D, C, X, P, B> Telemetry<Robot<D, C, X, P, Self, B>> for Autonomous<R> {
    fn record(_: &mut Robot<D, C, X, P, Self, B>, _: Sample) {}

    fn status(robot: &mut Robot<D, C, X, P, Self, B>, status: &str) {
        robot.autonomous.status(status);
    }
}

impl<R, D, C, X, P, S, B> Telemetry<Robot<D, C, X, P, Self, B>> for Autonomous<R, Sampler<S>>
where
    R: RouteSet,
    S: Report<Robot<D, C, X, P, Self, B>>,
    D: ComponentReport,
    X: ComponentReport,
{
    fn record(robot: &mut Robot<D, C, X, P, Self, B>, sample: Sample) {
        if let Some(frame) = robot.autonomous.sampler.frame(sample) {
            robot.drivetrain.report(frame);
            robot.mechanisms.report(frame);
            S::collect(robot, frame, sample);
        }
    }

    fn status(robot: &mut Robot<D, C, X, P, Self, B>, status: &str) {
        robot.autonomous.status(status);
    }

    fn driver_output(robot: &mut Robot<D, C, X, P, Self, B>, elapsed: Duration) {
        robot.autonomous.sampler.driver_output(elapsed);
    }
}

/// Displayed during ordinary driver control.
pub const DRIVER_STATUS: &str = "Driver control";

/// Whether an autonomous motion may continue issuing commands.
pub fn autonomous_active() -> bool {
    vexide::competition::mode() == CompetitionMode::Autonomous
}

pub(crate) fn autonomous_ui<R: RouteSet>(
    ui: Ui<Option<R>>,
    routes: impl IntoIterator<Item = R>,
) -> Autonomous<R> {
    Autonomous {
        menu: ui.build(
            None,
            routes
                .into_iter()
                .map(|route| (route.label().into(), Some(route)))
                .collect(),
        ),
        sampler: (),
        started: None,
        driver: None,
        last_driver_fault: None,
    }
}

#[cfg(test)]
mod tests;
