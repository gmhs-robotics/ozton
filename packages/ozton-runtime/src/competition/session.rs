//! Competition lifecycle and cancellation built on vexide's Compete runtime.

use std::{
    future::{Future, poll_fn},
    pin::{Pin, pin},
    task::Poll,
    time::{Duration, Instant},
};

use vexide::{
    competition::{self, Compete, CompeteExt, CompetitionMode},
    time::sleep,
};

use super::{CompetitionRobot, DRIVER_STATUS, RouteSet, Routes, Tick};
use crate::clock::CONTROL_PERIOD;

/// Vexide dispatches modes and cancels their futures. Ozton supplies the tick,
/// output cleanup, route selection and telemetry within each mode.
pub(crate) async fn compete<R: CompetitionRobot + Compete + 'static>(robot: R) -> ! {
    #[cfg(feature = "logging")]
    let _sd_writer = crate::diagnostics::start_sd_logging();
    crate::log_event!(
        Info,
        "competition",
        "Ready with {:.1} ms control period",
        R::CONTROL_PERIOD.as_secs_f64() * 1000.0
    );
    CompeteExt::compete(robot).await
}

pub(super) struct CompetitionSession<'a, R>(pub(super) &'a mut R);

impl<R: CompetitionRobot> CompetitionSession<'_, R> {
    pub(super) fn stop(&mut self) {
        self.0.set_autonomous_start(None);
        if let Err(error) = self.0.stop() {
            crate::log_event!(
                Error,
                "competition",
                "Could not stop robot outputs ({error})"
            );
        }
    }

    fn enter(&mut self, mode: CompetitionMode) {
        super::activation::advance();
        self.0.set_autonomous_start(None);
        // Enabled-mode handoff cancels execution, not latched device commands.
        // An actual disabled activation still explicitly clears outputs.
        if mode == CompetitionMode::Disabled {
            self.stop();
        }
        crate::log_event!(Info, "competition", "Entered {mode:?} mode");
        self.0.menu().poll(mode == CompetitionMode::Disabled);
        self.0.mode_changed(mode);
    }

    pub(super) async fn ticks(&mut self, mode: CompetitionMode) {
        let disabled = mode == CompetitionMode::Disabled;
        let mut previous = None;
        while competition::mode() == mode {
            let now = Instant::now();
            let tick = Tick {
                started: now,
                elapsed: previous.map_or(Duration::ZERO, |previous| {
                    now.saturating_duration_since(previous)
                }),
            };
            previous = Some(now);
            if disabled {
                let status = if self.0.selected_autonomous().is_some() {
                    "Ready for autonomous"
                } else {
                    "Autonomous disabled"
                };
                self.0.menu().status(status);
            }
            if mode == CompetitionMode::Driver {
                self.0.driver_tick(tick).await;
            } else {
                let _ = self.0.stop();
            }
            if competition::mode() != mode {
                break;
            }
            self.0.menu().poll(disabled);
            self.0.after_tick(mode, tick);
            sleep(control_sleep(R::CONTROL_PERIOD, tick.work())).await;
        }
    }
}

impl<R: CompetitionRobot> CompetitionSession<'_, R> {
    pub(super) async fn disabled(&mut self) {
        self.enter(CompetitionMode::Disabled);
        let _localization = self.0.start_localization(CompetitionMode::Disabled);
        self.0.idle_mode(CompetitionMode::Disabled).await;
    }

    pub(super) async fn driver(&mut self) {
        self.enter(CompetitionMode::Driver);
        self.0.menu().status(DRIVER_STATUS);
        let _localization = self.0.start_localization(CompetitionMode::Driver);
        let result = self.0.driver_mode().await;
        if competition::mode() != CompetitionMode::Driver {
            return;
        }
        if let Err(error) = result {
            self.stop();
            self.0.driver_failed(error);
        }
    }

    pub(super) async fn autonomous(&mut self) {
        let mode = CompetitionMode::Autonomous;
        self.enter(mode);
        let _localization = self.0.start_localization(mode);
        let robot = &mut *self.0;
        if let Some(route) = robot.selected_autonomous() {
            robot
                .menu()
                .status(&format!("Autonomous: {}", route.label()));
            crate::log_event!(Info, "autonomous", "Starting route {:?}", route.label());
            robot.set_autonomous_start(Some(Instant::now()));
            let result = while_in_mode(route.run(robot), mode, R::CONTROL_PERIOD).await;
            robot.set_autonomous_start(None);
            if competition::mode() != mode || result.is_none() {
                // Vexide drops the old callback on a transition. Do not issue
                // motor commands or update the next mode's status on exit.
                return;
            }
            match result {
                Some(Ok(())) => {
                    match robot.stop() {
                        Ok(()) => {
                            robot.menu().status("Autonomous complete");
                            crate::log_event!(Info, "autonomous", "Completed route {:?}", route.label());
                        }
                        Err(error) => robot.route_failed(route, error),
                    }
                }
                Some(Err(error)) => {
                    let stopped = robot.stop();
                    robot.route_failed(route, error);
                    if let Err(error) = stopped {
                        robot.route_failed(route, error);
                    }
                }
                None => return,
            }
        } else {
            robot.menu().status("Autonomous disabled");
        }
        self.0.idle_mode(mode).await;
    }
}

impl<D, C, X, P, A, B> Compete for crate::Robot<D, C, X, P, A, B>
where
    Self: CompetitionRobot,
{
    async fn connected(&mut self) {
        super::activation::advance();
        CompetitionSession(self).stop();
    }

    async fn disconnected(&mut self) {
        super::activation::advance();
        CompetitionSession(self).stop();
    }

    async fn disabled(&mut self) {
        CompetitionSession(self).disabled().await;
    }

    async fn driver(&mut self) {
        CompetitionSession(self).driver().await;
    }

    async fn autonomous(&mut self) {
        CompetitionSession(self).autonomous().await;
    }
}

pub(super) async fn while_in_mode<T>(
    route: impl Future<Output = T>,
    mode: CompetitionMode,
    period: Duration,
) -> Option<T> {
    let activation = super::activation::current();
    let mut route = pin!(route);
    let interval = period.clamp(Duration::from_millis(1), CONTROL_PERIOD);
    let mut mode_check = sleep(interval);
    poll_fn(|cx| {
        if !super::activation::is_current(activation, mode) {
            return Poll::Ready(None);
        }
        if let Poll::Ready(result) = route.as_mut().poll(cx) {
            return Poll::Ready(super::activation::is_current(activation, mode).then_some(result));
        }
        if Pin::new(&mut mode_check).poll(cx).is_ready() {
            mode_check = sleep(interval);
            let _ = Pin::new(&mut mode_check).poll(cx);
        }
        Poll::Pending
    })
    .await
}

pub(super) fn control_sleep(period: Duration, work: Duration) -> Duration {
    period.saturating_sub(work).max(Duration::from_millis(1))
}
