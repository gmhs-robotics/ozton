//! Competition lifecycle and cancellation built on vexide's Compete runtime.

use std::{
    convert::Infallible,
    future::{Future, poll_fn},
    ops::ControlFlow,
    pin::{Pin, pin},
    task::Poll,
    time::{Duration, Instant},
};

use vexide::{
    competition::{self, Compete, CompetitionMode, CompetitionRuntime},
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
    // Capture the connection identity when vexide creates the callback, not
    // when it first polls it. An unpolled callback may already be obsolete.
    match CompetitionRuntime::<R, Infallible, _, _, _, _, _>::builder(robot)
        .on_connect(|robot| Box::pin(async { ControlFlow::Continue(robot.connected().await) }))
        .on_disconnect(|robot| Box::pin(async { ControlFlow::Continue(robot.disconnected().await) }))
        .while_disabled(|robot| mode_future(robot, CompetitionMode::Disabled))
        .while_autonomous(|robot| mode_future(robot, CompetitionMode::Autonomous))
        .while_driving(|robot| mode_future(robot, CompetitionMode::Driver))
        .finish()
        .await
    {}
}

/// Vexide 0.9 polls the old callback once after observing a phase change.
/// Returning Ready here can make it incorrectly consume the connected or
/// disconnected phase as if that hook had completed. Quiesce and let vexide
/// drop the old callback instead.
async fn hold_mode(work: impl Future<Output = ()>, mut active: impl FnMut() -> bool) {
    let mut work = Some(Box::pin(work));
    poll_fn(|cx| {
        if !active() {
            work = None;
        } else if let Some(future) = &mut work {
            if future.as_mut().poll(cx).is_ready() {
                work = None;
            }
        }
        // Never signal completion of a mode callback to a different phase.
        Poll::<()>::Pending
    })
    .await
}

pub(super) async fn run_mode<R: CompetitionRobot>(
    robot: &mut R,
    mode: CompetitionMode,
    connected: bool,
) {
    let work = async {
        let mut session = CompetitionSession(robot);
        match mode {
            CompetitionMode::Disabled => session.disabled().await,
            CompetitionMode::Autonomous => session.autonomous().await,
            CompetitionMode::Driver => session.driver().await,
        }
    };
    hold_mode(work, || {
        let status = competition::status();
        status.mode() == mode && status.is_connected() == connected
    })
    .await;
}

fn mode_future<R: CompetitionRobot>(
    robot: &mut R,
    mode: CompetitionMode,
) -> Pin<Box<dyn Future<Output = ControlFlow<Infallible>> + '_>> {
    let connected = competition::is_connected();
    Box::pin(async move {
        run_mode(robot, mode, connected).await;
        ControlFlow::Continue(())
    })
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
        let token = super::activation::Token::capture(mode);
        let mut previous = None;
        while token.active() {
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
            if !token.active() {
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
        let _activation = super::activation::Activation::enter();
        self.enter(CompetitionMode::Disabled);
        let _localization = self.0.start_localization(CompetitionMode::Disabled);
        self.0.idle_mode(CompetitionMode::Disabled).await;
    }

    pub(super) async fn driver(&mut self) {
        let _activation = super::activation::Activation::enter();
        self.enter(CompetitionMode::Driver);
        self.0.menu().status(DRIVER_STATUS);
        let _localization = self.0.start_localization(CompetitionMode::Driver);
        let token = super::activation::Token::capture(CompetitionMode::Driver);
        let result = self.0.driver_mode().await;
        if !token.active() {
            return;
        }
        if let Err(error) = result {
            super::activation::advance();
            self.stop();
            self.0.driver_failed(error);
        }
    }

    pub(super) async fn autonomous(&mut self) {
        let _activation = super::activation::Activation::enter();
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
            super::activation::advance();
            match result {
                Some(Ok(())) => match robot.stop() {
                    Ok(()) => {
                        robot.menu().status("Autonomous complete");
                        crate::log_event!(
                            Info,
                            "autonomous",
                            "Completed route {:?}",
                            route.label()
                        );
                    }
                    Err(error) => robot.route_failed(route, error),
                },
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
        run_mode(self, CompetitionMode::Disabled, competition::is_connected()).await;
    }

    async fn driver(&mut self) {
        run_mode(self, CompetitionMode::Driver, competition::is_connected()).await;
    }

    async fn autonomous(&mut self) {
        run_mode(self, CompetitionMode::Autonomous, competition::is_connected()).await;
    }
}

pub(crate) async fn while_in_mode<T>(
    route: impl Future<Output = T>,
    mode: CompetitionMode,
    period: Duration,
) -> Option<T> {
    let token = super::activation::Token::capture(mode);
    let mut route = pin!(route);
    let interval = period.clamp(Duration::from_millis(1), CONTROL_PERIOD);
    let mut mode_check = sleep(interval);
    poll_fn(|cx| {
        if !token.active() {
            return Poll::Ready(None);
        }
        if let Poll::Ready(result) = route.as_mut().poll(cx) {
            return Poll::Ready(token.active().then_some(result));
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

#[cfg(all(test, not(target_os = "vexos")))]
mod tests {
    use std::{cell::Cell, task::{Context, Waker}};

    use super::*;

    #[test]
    fn obsolete_mode_never_polls_old_work_or_reports_completion() {
        struct OnDrop<'a>(&'a Cell<bool>);
        impl Drop for OnDrop<'_> {
            fn drop(&mut self) {
                self.0.set(true);
            }
        }

        let active = Cell::new(true);
        let polls = Cell::new(0);
        let dropped = Cell::new(false);
        let work = async {
            let _drop = OnDrop(&dropped);
            poll_fn(|_| {
                polls.set(polls.get() + 1);
                Poll::<()>::Pending
            })
            .await;
        };
        let mut callback = pin!(hold_mode(work, || active.get()));
        let mut cx = Context::from_waker(Waker::noop());
        assert!(callback.as_mut().poll(&mut cx).is_pending());
        assert_eq!(polls.get(), 1);
        active.set(false);
        assert!(callback.as_mut().poll(&mut cx).is_pending());
        assert!(dropped.get());
        active.set(true);
        assert!(callback.as_mut().poll(&mut cx).is_pending());
        assert_eq!(polls.get(), 1);
    }

    #[test]
    fn completed_mode_remains_quiescent_until_vexide_drops_it() {
        let entries = Cell::new(0);
        let mut callback = pin!(hold_mode(async { entries.set(entries.get() + 1); }, || true));
        let mut cx = Context::from_waker(Waker::noop());
        assert!(callback.as_mut().poll(&mut cx).is_pending());
        assert!(callback.as_mut().poll(&mut cx).is_pending());
        assert_eq!(entries.get(), 1);
    }
}
