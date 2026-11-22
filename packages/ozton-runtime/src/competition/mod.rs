//! Programmed route selection and a budgeted competition loop for custom robots.

use std::time::{Duration, Instant};

use ozton_ui::RouteSelector;
pub use ozton_ui::Ui;
pub use vexide::competition::Compete;
use vexide::{
    competition::{self, CompetitionMode},
    controller::Controller,
};

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
mod session;

pub use policy::{CompetitionLayout, Mechanism, Program};
pub use routes::*;
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

/// A driver routine may be preempted by controls or run out of time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriverWaitError {
    Cancelled,
    Timeout,
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

    /// Dispatch a driver binding after the ordinary tick.
    async fn driver_action(&mut self) {}

    fn driver_route_active(&self) -> bool {
        false
    }

    /// Release the active driver route's mechanism outputs before manual control resumes.
    fn cancel_driver_route(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Service driver outputs while a routine waits for its next condition.
    async fn driver_wait_until(
        &mut self,
        timeout: Duration,
        mut ready: impl FnMut(&Self) -> bool,
    ) -> Result<(), DriverWaitError> {
        let started = Instant::now();
        let mut clock = crate::clock::CommandClock::new(Self::CONTROL_PERIOD)
            .ok_or(DriverWaitError::Cancelled)?;
        loop {
            if competition::mode() != CompetitionMode::Driver || !self.driver_route_active() {
                return Err(DriverWaitError::Cancelled);
            }
            if ready(self) {
                return Ok(());
            }
            if started.elapsed() >= timeout {
                return Err(DriverWaitError::Timeout);
            }
            let elapsed = clock.tick().await;
            if competition::mode() != CompetitionMode::Driver {
                return Err(DriverWaitError::Cancelled);
            }
            let tick = Tick {
                started: Instant::now(),
                elapsed,
            };
            self.driver_tick(tick).await;
            self.after_tick(CompetitionMode::Driver, tick);
        }
    }

    /// Hold the currently commanded outputs for a duration while driver control continues.
    async fn driver_wait(&mut self, duration: Duration) -> Result<(), DriverWaitError> {
        let started = Instant::now();
        self.driver_wait_until(duration, |_| started.elapsed() >= duration)
            .await
    }

    fn mode_changed(&mut self, _mode: CompetitionMode) {}
    fn after_tick(&mut self, _mode: CompetitionMode, _tick: Tick) {}
    fn route_failed(&mut self, _route: Self::Route, error: Self::Error) {
        crate::diagnostic_event!("auton_error={error}");
        self.menu().status("Autonomous stopped");
    }
}

/// Brain selection and status for an assembled robot.
pub struct Autonomous<R: Copy + 'static, S = ()> {
    menu: Box<dyn RouteSelector<Option<R>>>,
    sampler: S,
    started: Option<Instant>,
    driver_routes: DriverRoutes<R>,
    driver: Option<R>,
}

struct DriverRoutes<R> {
    queued: Option<R>,
    active: Option<R>,
}

impl<R: Copy> DriverRoutes<R> {
    fn queue(&mut self, route: R) {
        if self.active.is_none() {
            self.queued = Some(route);
        }
    }

    fn start(&mut self) -> Option<R> {
        let route = self.queued.take()?;
        self.active = Some(route);
        Some(route)
    }

    fn finish(&mut self) -> bool {
        self.queued = None;
        self.active.take().is_some()
    }
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

    pub(crate) fn with_diagnostics<L>(self, interval: Duration) -> Autonomous<R, Sampler<L>> {
        Autonomous {
            menu: self.menu,
            sampler: Sampler::new(interval),
            started: self.started,
            driver_routes: self.driver_routes,
            driver: self.driver,
        }
    }
}

impl<R: RouteSet, S> Autonomous<R, S> {
    pub(crate) fn set_driver(&mut self, driver: R) {
        self.driver = Some(driver);
    }

    /// Schedule a registered route after this driver tick's outputs.
    pub fn queue_driver_route(&mut self, route: R) {
        self.driver_routes.queue(route);
    }

    pub fn selected_label(&self) -> Option<&'static str> {
        self.menu.action().map(RouteSet::label)
    }
}

impl<R: RouteSet, D, C, T, X, P> Telemetry<Robot<D, C, T, X, P, Self>> for Autonomous<R> {
    fn record(_: &mut Robot<D, C, T, X, P, Self>, _: Sample) {}

    fn status(robot: &mut Robot<D, C, T, X, P, Self>, status: &str) {
        robot.autonomous.status(status);
    }
}

impl<R, D, C, T, X, P, S> Telemetry<Robot<D, C, T, X, P, Self>> for Autonomous<R, Sampler<S>>
where
    R: RouteSet,
    S: Report<Robot<D, C, T, X, P, Self>>,
    D: ComponentReport,
    T: ComponentReport,
    X: ComponentReport,
{
    fn record(robot: &mut Robot<D, C, T, X, P, Self>, sample: Sample) {
        if let Some(frame) = robot.autonomous.sampler.frame(sample) {
            #[cfg(feature = "logging")]
            crate::diagnostics::report_controller(
                frame,
                robot.autonomous.sampler.controller.as_ref(),
            );
            robot.drivetrain.report(frame);
            robot.tracking.report(frame);
            robot.mechanisms.report(frame);
            S::collect(robot, frame, sample);
        }
    }

    fn status(robot: &mut Robot<D, C, T, X, P, Self>, status: &str) {
        robot.autonomous.status(status);
    }

    fn controller(
        robot: &mut Robot<D, C, T, X, P, Self>,
        state: Option<&vexide::controller::ControllerState>,
    ) {
        #[cfg(feature = "logging")]
        {
            robot.autonomous.sampler.controller = state.copied();
        }
        #[cfg(not(feature = "logging"))]
        let _ = (robot, state);
    }

    fn driver_output(robot: &mut Robot<D, C, T, X, P, Self>, elapsed: Duration) {
        robot.autonomous.sampler.driver_output(elapsed);
    }
}

/// Displayed during ordinary driver control.
pub const DRIVER_STATUS: &str = "Driver control";

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
        driver_routes: DriverRoutes {
            queued: None,
            active: None,
        },
    }
}

#[cfg(test)]
mod tests;
