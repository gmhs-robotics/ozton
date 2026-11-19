//! Programmed route selection and a budgeted competition loop for custom robots.

use std::time::{Duration, Instant};

use vexide::{
    competition::{self, CompetitionMode},
    display::Display,
    time::sleep,
};

use crate::{
    CustomList, Robot,
    clock::CONTROL_PERIOD,
    diagnostics::{Report, Sample, Sampler, Telemetry},
    menu::RouteMenu,
};

/// The available choices, shared by the Brain selector and route dispatch.
pub trait RouteSet: Copy + 'static {
    const CHOICES: &'static [(&'static str, Self)];

    fn label(self) -> &'static str;
}

/// The async routines for a route set. Define both traits with
/// [`crate::autonomous_routes`] alongside the routine functions.
#[allow(async_fn_in_trait)]
pub trait Routes<R, E>: RouteSet {
    async fn run(self, robot: &mut R) -> Result<(), E>;
}

/// Define the Brain choices and dispatch ordinary async functions. Different
/// route future types are compiled into one enum dispatch, with no boxed futures.
///
/// ```
/// use ozton_runtime::{Robot, autonomous_routes};
/// type PracticeBot = Robot<(), (), (), ((), usize)>;
/// async fn practice(robot: &mut PracticeBot) -> Result<(), ()> {
///     robot.custom.1 += 1;
///     Ok(())
/// }
/// autonomous_routes! {
///     enum Route for PracticeBot => () {
///         Practice("Practice") => practice,
///     }
/// }
/// ```
#[macro_export]
macro_rules! autonomous_routes {
    ($visibility:vis enum $name:ident for $robot:ty => $error:ty {
        $($variant:ident($label:literal) => $route:path),* $(,)?
    }) => {
        #[derive(Clone, Copy)]
        $visibility enum $name { $($variant),* }

        impl $crate::competition::RouteSet for $name {
            const CHOICES: &'static [(&'static str, Self)] = &[$(($label, Self::$variant)),*];

            fn label(self) -> &'static str {
                match self { $(Self::$variant => $label),* }
            }

        }

        impl $crate::competition::Routes<$robot, $error> for $name {
            #[allow(unused_variables)]
            async fn run(self, robot: &mut $robot) -> Result<(), $error> {
                match self { $(Self::$variant => $route(robot).await),* }
            }
        }
    };
}

/// Timing for the current control tick, including time spent in application hooks.
#[derive(Clone, Copy)]
pub struct Tick {
    started: Instant,
    pub elapsed: Duration,
}

impl Tick {
    pub fn work(self) -> Duration {
        self.started.elapsed()
    }
}

/// Opt-in competition hooks for an application with its own robot and controls.
/// Implementations own device errors and drive/motor policy. The loop owns mode
/// transitions, route selection, disable handling and the periodic tick.
pub trait CompetitionRobot: Sized {
    type Error: std::fmt::Display;
    type Route: Routes<Self, Self::Error>;
    const CONTROL_PERIOD: Duration = CONTROL_PERIOD;

    fn menu(&mut self) -> &mut RouteMenu<Option<Self::Route>>;
    fn stop(&mut self) -> Result<(), Self::Error>;
    fn driver_tick(&mut self, tick: Tick);

    fn mode_changed(&mut self, _mode: CompetitionMode) {}
    fn after_tick(&mut self, _mode: CompetitionMode, _tick: Tick) {}
    fn route_failed(&mut self, _route: Self::Route, error: Self::Error) {
        self.menu().status(&format!("Autonomous stopped: {error}"));
    }
}

/// Brain selection and status for an assembled robot.
pub struct Autonomous<R: Copy + 'static, S = ()> {
    menu: RouteMenu<Option<R>>,
    sampler: S,
}

impl<R: Copy + 'static, S> Autonomous<R, S> {
    pub fn status(&mut self, message: &str) {
        self.menu.status(message);
    }

    pub(crate) fn with_diagnostics<L>(self, interval: Duration) -> Autonomous<R, Sampler<L>> {
        Autonomous {
            menu: self.menu,
            sampler: Sampler::new(interval),
        }
    }
}

impl<R: RouteSet, D, C, T, X, P> Telemetry<Robot<D, C, T, X, P, Self>> for Autonomous<R> {
    fn record(_: &mut Robot<D, C, T, X, P, Self>, _: Sample) {}
}

impl<R, D, C, T, X, P, S> Telemetry<Robot<D, C, T, X, P, Self>> for Autonomous<R, Sampler<S>>
where
    R: RouteSet,
    S: Report<Robot<D, C, T, X, P, Self>>,
{
    fn record(robot: &mut Robot<D, C, T, X, P, Self>, sample: Sample) {
        if let Some(frame) = robot.autonomous.sampler.frame(sample) {
            S::collect(robot, frame, sample);
        }
    }

    fn driver_output(robot: &mut Robot<D, C, T, X, P, Self>, elapsed: Duration) {
        robot.autonomous.sampler.driver_output(elapsed);
    }
}

/// Application-specific competition behavior for an Ozton [`Robot`].
/// Install its state with `Robot::with_controls`. The hooks receive the
/// assembled robot and can borrow its concrete parts and controls independently.
pub trait CompetitionControls<R> {
    type Error: std::fmt::Display;
    type CustomInput;
    const CONTROL_PERIOD: Duration = CONTROL_PERIOD;

    /// Stop application-controlled outputs and clear application state.
    fn stop(robot: &mut R) -> Result<(), Self::Error>;
    /// Write driver outputs and supply this tick's shared custom input.
    /// Return None to skip the custom chain for this tick.
    fn driver_tick(robot: &mut R, tick: Tick) -> Option<Self::CustomInput>;

    fn after_custom_update(_robot: &mut R) {}
    fn mode_changed(_robot: &mut R, _mode: CompetitionMode) {}
    fn after_tick(_robot: &mut R, _mode: CompetitionMode, _tick: Tick) {}
    fn autonomous_error(
        _robot: &mut R,
        _name: &'static str,
        _error: &Self::Error,
    ) -> Option<&'static str> {
        None
    }
}

impl<D, C, T, X, P, R, S> CompetitionRobot for Robot<D, C, T, X, P, Autonomous<R, S>>
where
    P: CompetitionControls<Self>,
    X: CustomList<P::CustomInput, P::Error>,
    R: Routes<Self, P::Error>,
    Autonomous<R, S>: Telemetry<Self>,
{
    type Error = P::Error;
    type Route = R;
    const CONTROL_PERIOD: Duration = P::CONTROL_PERIOD;

    fn menu(&mut self) -> &mut RouteMenu<Option<R>> {
        &mut self.autonomous.menu
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        P::stop(self).and(self.custom.stop_all())
    }

    fn driver_tick(&mut self, tick: Tick) {
        if let Some(mut input) = P::driver_tick(self, tick) {
            match self.custom.update_all(&mut input) {
                Ok(()) => P::after_custom_update(self),
                Err(_) => {
                    let _ = self.stop();
                }
            }
        }
    }

    fn mode_changed(&mut self, mode: CompetitionMode) {
        P::mode_changed(self, mode);
    }

    fn after_tick(&mut self, mode: CompetitionMode, tick: Tick) {
        P::after_tick(self, mode, tick);
        self.record_diagnostics(Sample {
            mode,
            dt: tick.elapsed,
            motion: None,
        });
    }

    fn route_failed(&mut self, route: Self::Route, error: Self::Error) {
        if let Some(status) = P::autonomous_error(self, route.label(), &error) {
            self.autonomous.status(status);
        } else {
            self.autonomous
                .status(&format!("Autonomous stopped: {error}"));
        }
    }
}

/// Build the Brain menu once, with Disable selected before any user route.
pub fn autonomous_menu<R: RouteSet>(display: Display) -> Autonomous<R> {
    Autonomous {
        menu: RouteMenu::with_choices(
            display,
            None,
            R::CHOICES
                .iter()
                .map(|&(label, route)| (label, Some(route))),
        ),
        sampler: (),
    }
}

/// Menu construction for a custom robot using [`CompetitionRobot`] directly.
pub fn route_menu<R: CompetitionRobot>(display: Display) -> RouteMenu<Option<R::Route>> {
    autonomous_menu(display).menu
}

/// Run the common mode loop. Route code that drives over multiple ticks should
/// use Ozton's runner or otherwise check disable itself.
pub async fn compete<R: CompetitionRobot>(mut robot: R) -> ! {
    let mut previous_mode = None;
    let mut last_tick = Instant::now();
    loop {
        let now = Instant::now();
        let tick = Tick {
            started: now,
            elapsed: now.saturating_duration_since(last_tick),
        };
        last_tick = now;
        let mode = competition::mode();
        let disabled = mode == CompetitionMode::Disabled;
        let entering = previous_mode != Some(mode);
        if entering {
            let _ = robot.stop();
            previous_mode = Some(mode);
            robot.menu().poll(disabled);
            robot.mode_changed(mode);
            if mode == CompetitionMode::Driver {
                robot.menu().status("Driver control");
            }
        }
        if competition::mode() != mode {
            let _ = robot.stop();
            sleep(Duration::from_millis(1)).await;
            continue;
        }
        match mode {
            CompetitionMode::Disabled => {
                let _ = robot.stop();
                let status = if robot.menu().action().is_some() {
                    "Ready for autonomous"
                } else {
                    "Autonomous disabled"
                };
                robot.menu().status(status);
            }
            CompetitionMode::Driver => robot.driver_tick(tick),
            CompetitionMode::Autonomous if entering => {
                if let Some(route) = robot.menu().action() {
                    let result = match robot.stop() {
                        Ok(()) => route.run(&mut robot).await,
                        Err(error) => Err(error),
                    };
                    match result.and(robot.stop()) {
                        Ok(()) => robot.menu().status("Autonomous complete"),
                        Err(error) => robot.route_failed(route, error),
                    }
                } else {
                    robot.menu().status("Autonomous disabled");
                }
                // The route owned its control ticks. Sample competition mode again.
                last_tick = Instant::now();
                continue;
            }
            CompetitionMode::Autonomous => {
                let _ = robot.stop();
            }
        }
        robot.menu().poll(disabled);
        robot.after_tick(mode, tick);
        sleep(control_sleep(R::CONTROL_PERIOD, tick.work())).await;
    }
}

fn control_sleep(period: Duration, work: Duration) -> Duration {
    period.saturating_sub(work).max(Duration::from_millis(1))
}

#[cfg(test)]
mod tests {
    use std::{
        future::Future,
        pin::pin,
        task::{Context, Poll, Waker},
    };

    use super::*;

    type TestRobot = Robot<(), (), (), (), i32>;

    async fn add(robot: &mut TestRobot) -> Result<(), &'static str> {
        robot.controls += 2;
        Ok(())
    }

    async fn multiply(robot: &mut TestRobot) -> Result<(), &'static str> {
        robot.controls *= 3;
        Err("test fault")
    }

    autonomous_routes! {
        enum TestRoute for TestRobot => &'static str {
            Add("Add") => add,
            Multiply("Multiply") => multiply,
        }
    }

    #[test]
    fn macro_registers_and_dispatches_distinct_route_futures() {
        let mut robot = Robot::new().with_controls(1);
        let mut context = Context::from_waker(Waker::noop());
        assert_eq!(TestRoute::CHOICES[0].0, "Add");
        assert_eq!(TestRoute::CHOICES[1].1.label(), "Multiply");
        assert_eq!(
            pin!(TestRoute::Add.run(&mut robot)).poll(&mut context),
            Poll::Ready(Ok(()))
        );
        assert_eq!(
            pin!(TestRoute::Multiply.run(&mut robot)).poll(&mut context),
            Poll::Ready(Err("test fault"))
        );
        assert_eq!(robot.controls, 9);
    }

    #[test]
    fn control_period_includes_work_and_overruns_still_yield() {
        let period = Duration::from_millis(10);
        assert_eq!(control_sleep(period, Duration::ZERO), period);
        assert_eq!(
            control_sleep(period, Duration::from_millis(3)),
            Duration::from_millis(7)
        );
        assert_eq!(control_sleep(period, period), Duration::from_millis(1));
        assert_eq!(
            control_sleep(period, Duration::from_secs(1)),
            Duration::from_millis(1)
        );
    }
}
