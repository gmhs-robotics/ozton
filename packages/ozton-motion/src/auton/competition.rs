//! One programmed route selection per autonomous period.

use std::{fmt::Debug, time::Duration};

use ozton_runtime::{
    clock::{CONTROL_PERIOD, CommandClock},
    selector::{Selector, SelectorItem, StatusHandle},
};
use vexide::{
    competition::{Compete, CompeteExt},
    display::Display,
    time::sleep,
};

use super::{AutonError, AutonRobot};

#[allow(async_fn_in_trait)]
pub trait ProgrammedRobot: AutonRobot {
    type Route: Copy + 'static;

    async fn run_route(&mut self, route: Self::Route) -> Result<(), AutonError<Self::Error>>;
    fn driver_tick(&mut self, dt: Duration);
    /// Invoked once on entry to disabled, after all outputs have been stopped.
    fn on_disabled(&mut self) {}
    fn tracking_status(&self) -> Option<&'static str> {
        None
    }
}

#[derive(Clone)]
struct Choice<I> {
    label: &'static str,
    route: Option<I>,
}

impl<I: Copy> SelectorItem for Choice<I> {
    fn label(&self) -> &str {
        self.label
    }
}

/// Starts competition control with Disable selected. Routes are chosen on the Brain.
pub async fn compete<R: ProgrammedRobot + 'static>(
    robot: R,
    display: Display,
    routes: &[(&'static str, R::Route)],
) -> !
where
    R::Error: Debug,
{
    compete_app(robot, display, routes).await
}

pub(super) trait CompetitionApp {
    type Route: Copy + 'static;
    type Error: Debug;

    fn stop(&mut self);
    fn driver(&mut self, selected: Option<Self::Route>, dt: Duration) -> Option<&'static str>;
    async fn run(&mut self, route: Self::Route) -> Result<(), AutonError<Self::Error>>;
    fn allows_autonomous(&self, _route: Self::Route) -> bool {
        true
    }
    fn disabled(&mut self) -> Option<&'static str>;
    fn status(&self) -> Option<&'static str>;
}

impl<R: ProgrammedRobot> CompetitionApp for R
where
    R::Error: Debug,
{
    type Route = R::Route;
    type Error = R::Error;

    fn stop(&mut self) {
        let _ = AutonRobot::stop(self);
    }
    fn driver(&mut self, _: Option<Self::Route>, dt: Duration) -> Option<&'static str> {
        self.driver_tick(dt);
        None
    }
    async fn run(&mut self, route: Self::Route) -> Result<(), AutonError<Self::Error>> {
        self.run_route(route).await
    }
    fn disabled(&mut self) -> Option<&'static str> {
        self.on_disabled();
        None
    }
    fn status(&self) -> Option<&'static str> {
        self.tracking_status()
    }
}

pub(super) async fn compete_app<R: CompetitionApp + 'static>(
    robot: R,
    display: Display,
    routes: &[(&'static str, R::Route)],
) -> ! {
    let mut choices = vec![Choice {
        label: "Disable",
        route: None,
    }];
    choices.extend(routes.iter().map(|&(label, route)| Choice {
        label,
        route: Some(route),
    }));
    let selector = Selector::new(display, choices, 0);
    Competition {
        robot,
        status: selector.status_handle(),
        _selector: selector,
    }
    .compete()
    .await
}

struct Competition<R: CompetitionApp> {
    robot: R,
    status: StatusHandle<Choice<R::Route>>,
    _selector: Selector<Choice<R::Route>>,
}

impl<R: CompetitionApp> Compete for Competition<R> {
    async fn driver(&mut self) {
        self.robot.stop();
        let route = self.status.with_selection(|choice| choice.route);
        let mut clock = CommandClock::new(CONTROL_PERIOD).expect("nonzero control period");
        let mut shown = None;
        loop {
            let dt = clock.tick().await;
            if let Some(message) = self.robot.driver(route, dt)
                && shown != Some(message)
            {
                self.status.show_status_persistent(message);
                shown = Some(message);
            }
        }
    }

    async fn autonomous(&mut self) {
        self.robot.stop();
        if let Some(route) = self
            .status
            .selection()
            .route
            .filter(|route| self.robot.allows_autonomous(*route))
        {
            self.status.show_status_persistent("Autonomous running");
            let message = match self.robot.run(route).await {
                Ok(()) => match self.robot.status() {
                    Some(status) => format!("Autonomous complete | {status}"),
                    None => "Autonomous complete".to_owned(),
                },
                Err(error) => format!("Autonomous stopped: {error:?}"),
            };
            self.status.show_status_persistent(message);
        } else {
            self.status.show_status_persistent("Autonomous disabled");
        }
        self.idle().await;
    }

    async fn disabled(&mut self) {
        self.robot.stop();
        if let Some(message) = self.robot.disabled() {
            self.status.show_status_persistent(message);
        }
        self.idle().await;
    }

    async fn disconnected(&mut self) {
        self.robot.stop();
    }
}

impl<R: CompetitionApp> Competition<R> {
    async fn idle(&mut self) {
        loop {
            self.robot.stop();
            sleep(CONTROL_PERIOD).await;
        }
    }
}
