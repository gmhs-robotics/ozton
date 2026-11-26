//! Competition dispatch for a robot assembled by the application.

use std::{fmt::Display, time::Duration};

use ozton_ui::{RouteSelector, RouteUi};
use vexide::competition::CompetitionMode;

use super::{RobotRoute, selector_task::TaskSelector, session};
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

    /// Autonomous routines offered by the Brain selector, in menu order.
    const ROUTES: &'static [RobotRoute<Self>] = &[];

    /// Initial Brain menu selection by label. An absent or unknown label selects Disable.
    const DEFAULT_ROUTE: Option<&'static str> = None;

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
    pub(super) robot: R,
    pub(super) selector: TaskSelector<Option<RobotRoute<R>>>,
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
    pub fn with_ui(mut self, ui: impl RouteUi<Option<RobotRoute<R>>>) -> Self {
        let routes = R::ROUTES.iter().map(|route| (route.label, Some(*route)));
        let default = R::DEFAULT_ROUTE
            .and_then(|label| R::ROUTES.iter().position(|route| route.label == label));
        self.selector = TaskSelector::new(Box::new(ui.into_selector(None, routes, default)));
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
    pub(super) fn mode_changed(&mut self, mode: CompetitionMode) {
        if mode == CompetitionMode::Disabled {
            self.selector.status("Select autonomous route");
        }
        if let Some(localization) = &self.localization {
            localization.mode_changed(mode);
        }
        self.robot.mode_changed(mode);
    }

    pub(super) fn route_failed(&mut self, label: &str, error: R::Error) {
        crate::log_event!(Error, "autonomous", "Route {label:?} stopped ({error})");
        self.selector.status("Autonomous stopped");
    }
}

#[cfg(all(test, not(target_os = "vexos")))]
mod tests;
