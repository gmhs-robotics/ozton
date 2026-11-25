//! Competition dispatch for a robot assembled by the application.

use std::{fmt::Display, time::Duration};

use ozton_ui::RouteSelector;
use vexide::{competition::CompetitionMode, display::Display as BrainDisplay};

use super::{Compete, RobotRoute, Ui, selector_task::TaskSelector, session};
use crate::{clock::CONTROL_PERIOD, localization::LocalizationWorker};

/// Hardware and behavior supplied by an application to the competition runtime.
#[allow(async_fn_in_trait)]
pub trait OztonCompete: Sized + 'static {
    type Error: Display + 'static;

    const CONTROL_PERIOD: Duration = CONTROL_PERIOD;

    /// Stop outputs after disable, connection changes, completion or failure.
    /// A direct enabled-mode handoff does not call this hook.
    fn stop(&mut self) -> Result<(), Self::Error>;

    /// Stop after a control fault. Robots with active braking should override this.
    fn fault_stop(&mut self) -> Result<(), Self::Error> {
        self.stop()
    }

    const ROUTES: &'static [RobotRoute<Self, Self::Error>] = &[];

    /// Initial Brain menu selection. An absent or unknown label selects Disable.
    const DEFAULT_ROUTE: Option<&'static str> = None;

    /// Create the localization worker once, after hardware initialization.
    fn start_localization(&self) -> Option<LocalizationWorker> {
        None
    }

    /// Run for one driver activation. The application owns its loop and yields between commands.
    async fn driver(&mut self) -> Result<(), Self::Error>;

    /// Reset application-owned transient state without writing motor outputs.
    fn mode_changed(&mut self, _mode: CompetitionMode) {}

    /// Install the Brain route selector and prepare this robot to compete.
    fn with_brain(self, display: BrainDisplay) -> Runtime<Self> {
        Runtime::new(self).with_ui(Ui::Brain(display))
    }
}

/// Runs competition lifecycle and owns its selector.
pub struct Runtime<R: OztonCompete> {
    pub(super) robot: R,
    pub(super) selector: TaskSelector<Option<RobotRoute<R, R::Error>>>,
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
        self.selector = TaskSelector::new(ui.build_with_default(None, routes, R::DEFAULT_ROUTE));
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

impl<R: OztonCompete> Runtime<R> {
    pub(super) fn selected_autonomous(&self) -> Option<RobotRoute<R, R::Error>> {
        self.selector
            .action()
            .filter(|route| route.allows_autonomous())
    }

    pub(super) fn mode_changed(&mut self, mode: CompetitionMode) {
        if mode == CompetitionMode::Disabled {
            self.selector.status("Select autonomous route");
        }
        if let Some(localization) = &self.localization {
            localization.mode_changed(mode);
        }
        self.robot.mode_changed(mode);
    }

    pub(super) fn route_failed(&mut self, route: RobotRoute<R, R::Error>, error: R::Error) {
        crate::log_event!(
            Error,
            "autonomous",
            "Route {:?} stopped ({error})",
            route.label()
        );
        self.selector.status("Autonomous stopped");
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
