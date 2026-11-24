//! Competition dispatch for a robot assembled by the application.

use std::{fmt::Display, future::pending, time::Duration};

use super::{
    Compete, CompetitionMode, CompetitionRobot, RobotRoute, RouteSelector, RouteSet, Routes, Tick,
    Ui, selector_task::TaskSelector, session, while_in_mode,
};
use crate::{clock::CONTROL_PERIOD, localization::LocalizationWorker};

/// Hardware and behavior supplied by an application to the competition runtime.
#[allow(async_fn_in_trait)]
pub trait OztonCompete: Sized + 'static {
    type Error: Display + 'static;

    const CONTROL_PERIOD: Duration = CONTROL_PERIOD;

    /// Stop outputs after disable, connection changes, completion or failure.
    /// A direct enabled-mode handoff does not call this hook.
    fn stop(&mut self) -> Result<(), Self::Error>;

    const ROUTES: &'static [RobotRoute<Self, Self::Error>] = &[];

    /// Create the localization worker once, after hardware initialization.
    fn start_localization(&self) -> Option<LocalizationWorker> {
        None
    }

    /// Run for one driver activation. The application owns its loop and yields between commands.
    async fn driver(&mut self) -> Result<(), Self::Error>;

    /// Reset application-owned transient state without writing motor outputs.
    fn mode_changed(&mut self, _mode: CompetitionMode) {}
}

/// Runs competition lifecycle and owns its selector.
pub struct Runtime<R: OztonCompete> {
    robot: R,
    selector: TaskSelector<Option<RobotRoute<R, R::Error>>>,
    localization: Option<LocalizationWorker>,
}

impl<R: OztonCompete> Runtime<R> {
    /// Create a runtime with autonomous selection disabled until a UI is supplied.
    pub fn new(robot: R) -> Self {
        Self {
            robot,
            selector: TaskSelector::new(Box::new(DisabledSelector)),
            localization: None,
        }
    }

    /// Give the runtime a selector for the robot's registered routes.
    #[must_use]
    pub fn with_ui(mut self, ui: Ui<Option<RobotRoute<R, R::Error>>>) -> Self {
        let routes = R::ROUTES
            .iter()
            .map(|route| (route.label().into(), Some(*route)))
            .collect();
        self.selector = TaskSelector::new(ui.build(None, routes));
        self
    }

    pub async fn compete(mut self) -> ! {
        self.localization = self.robot.start_localization();
        let _selector_task = self.selector.start();
        session::compete(self).await
    }
}

struct DisabledSelector;

impl<A> RouteSelector<Option<A>> for DisabledSelector {
    fn action(&self) -> Option<A> {
        None
    }

    fn poll(&mut self, _: bool) -> Option<Option<A>> {
        None
    }

    fn status(&mut self, _: &str) {}
}

impl<R: OztonCompete> Routes<Runtime<R>, R::Error> for RobotRoute<R, R::Error> {
    async fn run(self, runtime: &mut Runtime<R>) -> Result<(), R::Error> {
        Routes::run(self, &mut runtime.robot).await
    }

    fn allows_autonomous(self) -> bool {
        <Self as Routes<R, R::Error>>::allows_autonomous(self)
    }
}

impl<R: OztonCompete> CompetitionRobot for Runtime<R> {
    type Error = R::Error;
    type Route = RobotRoute<R, R::Error>;
    const CONTROL_PERIOD: Duration = R::CONTROL_PERIOD;

    fn menu(&mut self) -> &mut dyn RouteSelector<Option<Self::Route>> {
        &mut self.selector
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.robot.stop()
    }

    async fn driver_tick(&mut self, _: Tick) {}

    async fn idle_mode(&mut self, mode: CompetitionMode) {
        while_in_mode(pending::<()>(), mode, R::CONTROL_PERIOD).await;
    }

    async fn driver_mode(&mut self) -> Result<(), Self::Error> {
        while_in_mode(
            self.robot.driver(),
            CompetitionMode::Driver,
            R::CONTROL_PERIOD,
        )
        .await
        .unwrap_or(Ok(()))
    }

    fn mode_changed(&mut self, mode: CompetitionMode) {
        if mode == CompetitionMode::Disabled {
            self.selector.status("Select autonomous route");
        }
        if let Some(localization) = &self.localization {
            localization.mode_changed(mode);
        }
        self.robot.mode_changed(mode);
    }
}

impl<R: OztonCompete> Compete for Runtime<R> {
    async fn connected(&mut self) {
        super::activation::advance();
        session::CompetitionSession(self).stop();
    }

    async fn disconnected(&mut self) {
        super::activation::advance();
        session::CompetitionSession(self).stop();
    }

    async fn disabled(&mut self) {
        session::run_mode(
            self,
            CompetitionMode::Disabled,
            vexide::competition::is_connected(),
        )
        .await;
    }

    async fn driver(&mut self) {
        session::run_mode(
            self,
            CompetitionMode::Driver,
            vexide::competition::is_connected(),
        )
        .await;
    }

    async fn autonomous(&mut self) {
        session::run_mode(
            self,
            CompetitionMode::Autonomous,
            vexide::competition::is_connected(),
        )
        .await;
    }
}

#[cfg(all(test, not(target_os = "vexos")))]
mod tests;
