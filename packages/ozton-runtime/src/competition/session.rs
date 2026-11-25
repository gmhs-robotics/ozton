//! Competition lifecycle and cancellation built on vexide's Compete runtime.

use std::{
    convert::Infallible,
    future::{Future, pending, poll_fn},
    ops::ControlFlow,
    pin::{Pin, pin},
    task::Poll,
    time::Duration,
};

use ozton_ui::RouteSelector;
use vexide::{
    competition::{self, Compete, CompetitionMode, CompetitionRuntime},
    time::sleep,
};

use super::{OztonCompete, Runtime};
use crate::clock::CONTROL_PERIOD;

/// Vexide dispatches modes and cancels their futures. Ozton supplies the tick,
/// output cleanup, route selection and telemetry within each mode.
pub(crate) async fn compete<R: OztonCompete>(robot: Runtime<R>) -> ! {
    let _startup = crate::startup::Startup::begin();
    #[cfg(feature = "logging")]
    let _diagnostics = crate::diagnostics::start_diagnostics();
    crate::log_event!(
        Info,
        "competition",
        "Ready with {:.1} ms control period",
        R::CONTROL_PERIOD.as_secs_f64() * 1000.0
    );
    // Capture the connection identity when vexide creates the callback, not
    // when it first polls it. An unpolled callback may already be obsolete.
    match CompetitionRuntime::<Runtime<R>, Infallible, _, _, _, _, _>::builder(robot)
        .on_connect(|robot| Box::pin(async { ControlFlow::Continue(robot.connected().await) }))
        .on_disconnect(|robot| {
            Box::pin(async { ControlFlow::Continue(robot.disconnected().await) })
        })
        .while_disabled(|robot| mode_future(robot, CompetitionMode::Disabled))
        .while_autonomous(|robot| mode_future(robot, CompetitionMode::Autonomous))
        .while_driving(|robot| mode_future(robot, CompetitionMode::Driver))
        .finish()
        .await {}
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

pub(super) async fn run_mode<R: OztonCompete>(
    robot: &mut Runtime<R>,
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

fn mode_future<R: OztonCompete>(
    robot: &mut Runtime<R>,
    mode: CompetitionMode,
) -> Pin<Box<dyn Future<Output = ControlFlow<Infallible>> + '_>> {
    let connected = competition::is_connected();
    Box::pin(async move {
        run_mode(robot, mode, connected).await;
        ControlFlow::Continue(())
    })
}

pub(super) struct CompetitionSession<'a, R: OztonCompete>(pub(super) &'a mut Runtime<R>);

impl<R: OztonCompete> CompetitionSession<'_, R> {
    pub(super) fn stop(&mut self) {
        if let Err(_error) = self.0.robot.stop() {
            crate::log_event!(
                Error,
                "competition",
                "Could not stop robot outputs ({_error})"
            );
        }
    }

    fn enter(&mut self, mode: CompetitionMode) {
        // Enabled-mode handoff cancels execution, not latched device commands.
        if mode == CompetitionMode::Disabled {
            self.stop();
        }
        crate::log_event!(Info, "competition", "Entered {mode:?} mode");
        self.0.selector.poll(mode == CompetitionMode::Disabled);
        self.0.mode_changed(mode);
    }

    pub(super) async fn disabled(&mut self) {
        let _activation = super::activation::Activation::enter();
        self.enter(CompetitionMode::Disabled);
        while_in_mode(
            pending::<()>(),
            CompetitionMode::Disabled,
            R::CONTROL_PERIOD,
        )
        .await;
    }

    pub(super) async fn driver(&mut self) {
        let mut activation = super::activation::Activation::enter();
        self.enter(CompetitionMode::Driver);
        self.0.selector.status("Driver control");
        let result = while_in_mode(
            self.0.robot.driver(),
            CompetitionMode::Driver,
            R::CONTROL_PERIOD,
        )
        .await;
        if let Some(Err(error)) = result {
            activation.renew();
            if let Err(_error) = self.0.robot.fault_stop() {
                crate::log_event!(Error, "competition", "Could not stop robot outputs ({_error})");
            }
            crate::log_event!(Error, "driver", "Control fault ({error})");
            self.0.selector.status(&error.to_string());
        }
    }

    pub(super) async fn autonomous(&mut self) {
        let mut activation = super::activation::Activation::enter();
        let mode = CompetitionMode::Autonomous;
        self.enter(mode);
        let runtime = &mut *self.0;
        if let Some(route) = runtime.selected_autonomous() {
            runtime
                .selector
                .status(&format!("Autonomous: {}", route.label()));
            crate::log_event!(Info, "autonomous", "Starting route {:?}", route.label());
            let result =
                while_in_mode(route.run(&mut runtime.robot), mode, R::CONTROL_PERIOD).await;
            if competition::mode() != mode || result.is_none() {
                // Do not write motor outputs or status for an obsolete activation.
                return;
            }
            activation.renew();
            match result {
                Some(Ok(())) => match runtime.robot.stop() {
                    Ok(()) => {
                        runtime.selector.status("Autonomous complete");
                        crate::log_event!(
                            Info,
                            "autonomous",
                            "Completed route {:?}",
                            route.label()
                        );
                    }
                    Err(error) => runtime.route_failed(route, error),
                },
                Some(Err(error)) => {
                    let stopped = runtime.robot.fault_stop();
                    runtime.route_failed(route, error);
                    if let Err(error) = stopped {
                        runtime.route_failed(route, error);
                    }
                }
                None => return,
            }
        } else {
            runtime.selector.status("Autonomous disabled");
        }
        while_in_mode(pending::<()>(), mode, R::CONTROL_PERIOD).await;
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

#[cfg(all(test, not(target_os = "vexos")))]
mod tests {
    use std::{
        cell::Cell,
        task::{Context, Waker},
    };

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
        let mut callback = pin!(hold_mode(
            async {
                entries.set(entries.get() + 1);
            },
            || true
        ));
        let mut cx = Context::from_waker(Waker::noop());
        assert!(callback.as_mut().poll(&mut cx).is_pending());
        assert!(callback.as_mut().poll(&mut cx).is_pending());
        assert_eq!(entries.get(), 1);
    }
}
