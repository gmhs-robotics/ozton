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

struct CompetitionSession<'a, R>(&'a mut R);

impl<R: CompetitionRobot> CompetitionSession<'_, R> {
    fn stop(&mut self) {
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
        self.stop();
        crate::log_event!(Info, "competition", "Entered {mode:?} mode");
        self.0.menu().poll(mode == CompetitionMode::Disabled);
        self.0.mode_changed(mode);
    }

    async fn ticks(&mut self, mode: CompetitionMode) {
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
        self.stop();
    }
}

impl<R: CompetitionRobot> CompetitionSession<'_, R> {
    async fn disabled(&mut self) {
        self.enter(CompetitionMode::Disabled);
        self.ticks(CompetitionMode::Disabled).await;
    }

    async fn driver(&mut self) {
        self.enter(CompetitionMode::Driver);
        self.0.menu().status(DRIVER_STATUS);
        self.ticks(CompetitionMode::Driver).await;
    }

    async fn autonomous(&mut self) {
        let mode = CompetitionMode::Autonomous;
        self.enter(mode);
        let robot = &mut *self.0;
        if let Some(route) = robot.selected_autonomous() {
            robot
                .menu()
                .status(&format!("Autonomous: {}", route.label()));
            crate::log_event!(Info, "autonomous", "Starting route {:?}", route.label());
            let result = match robot.stop() {
                Ok(()) => {
                    robot.set_autonomous_start(Some(Instant::now()));
                    while_in_mode(
                        route.run(robot),
                        CompetitionMode::Autonomous,
                        R::CONTROL_PERIOD,
                    )
                    .await
                }
                Err(error) => Some(Err(error)),
            };
            robot.set_autonomous_start(None);
            let stopped = robot.stop();
            match (result, stopped) {
                (Some(Ok(())), Ok(())) => {
                    robot.menu().status("Autonomous complete");
                    crate::log_event!(Info, "autonomous", "Completed route {:?}", route.label());
                }
                (None, Ok(())) => {
                    robot.menu().status("Autonomous cancelled");
                    crate::log_event!(
                        Warn,
                        "autonomous",
                        "Route {:?} cancelled after mode change",
                        route.label()
                    );
                }
                (Some(Err(error)), Err(stop)) => {
                    robot.route_failed(route, error);
                    robot.route_failed(route, stop);
                }
                (Some(Err(error)), Ok(())) | (_, Err(error)) => {
                    robot.route_failed(route, error);
                }
            }
        } else {
            robot.menu().status("Autonomous disabled");
        }
        self.ticks(mode).await;
    }
}

impl<D, C, X, P, A, B> Compete for crate::Robot<D, C, X, P, A, B>
where
    Self: CompetitionRobot,
{
    async fn connected(&mut self) {
        CompetitionSession(self).stop();
    }

    async fn disconnected(&mut self) {
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
    let mut route = pin!(route);
    let interval = period.clamp(Duration::from_millis(1), CONTROL_PERIOD);
    let mut mode_check = sleep(interval);
    poll_fn(|cx| {
        if competition::mode() != mode {
            return Poll::Ready(None);
        }
        if let Poll::Ready(result) = route.as_mut().poll(cx) {
            return Poll::Ready(Some(result));
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
