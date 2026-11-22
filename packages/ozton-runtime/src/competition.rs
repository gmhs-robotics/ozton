//! Programmed route selection and a budgeted competition loop for custom robots.

use std::{
    future::{Future, poll_fn},
    pin::{Pin, pin},
    task::Poll,
    time::{Duration, Instant},
};

use ozton_drivetrain::model::ProgramDrive;
use vexide::{
    competition::{self, CompetitionMode},
    controller::{Controller, ControllerState},
    display::Display,
    time::sleep,
};

use crate::{
    Robot,
    clock::CONTROL_PERIOD,
    diagnostics::{Report, Sample, Sampler, Telemetry},
    menu::RouteMenu,
};

/// The available choices, shared by the Brain selector and route dispatch.
pub trait RouteSet: Copy + 'static {
    fn label(self) -> &'static str;
}

/// Dispatch the selected async routine.
#[allow(async_fn_in_trait)]
pub trait Routes<R, E>: RouteSet {
    async fn run(self, robot: &mut R) -> Result<(), E>;
    fn allows_autonomous(self) -> bool {
        true
    }
}

#[doc(hidden)]
pub trait IntoRouteResult<E> {
    fn into_route_result(self) -> Result<(), E>;
}

impl<E> IntoRouteResult<E> for () {
    fn into_route_result(self) -> Result<(), E> {
        Ok(())
    }
}

impl<E, F: Into<E>> IntoRouteResult<E> for Result<(), F> {
    fn into_route_result(self) -> Result<(), E> {
        self.map_err(Into::into)
    }
}

type RouteHandler<L, E> = for<'a> fn(
    &'a mut crate::CompetitionBot<L>,
) -> Pin<Box<dyn Future<Output = Result<(), E>> + 'a>>;

/// One named async routine registered with the robot builder.
pub struct RegisteredRoute<L: crate::RobotLayout, E> {
    label: &'static str,
    handler: Option<RouteHandler<L, E>>,
}

impl<L: crate::RobotLayout, E> RegisteredRoute<L, E> {
    #[doc(hidden)]
    pub const fn new(label: &'static str, handler: RouteHandler<L, E>) -> Self {
        Self {
            label,
            handler: Some(handler),
        }
    }

    /// A selectable driver tool, such as a supervised calibration trial.
    pub const fn driver_only(label: &'static str) -> Self {
        Self {
            label,
            handler: None,
        }
    }
}

impl<L: crate::RobotLayout, E> Copy for RegisteredRoute<L, E> {}
impl<L: crate::RobotLayout, E> Clone for RegisteredRoute<L, E> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<L: crate::RobotLayout + 'static, E: 'static> RouteSet for RegisteredRoute<L, E> {
    fn label(self) -> &'static str {
        self.label
    }
}

impl<L: crate::RobotLayout + 'static, E: 'static> Routes<crate::CompetitionBot<L>, E>
    for RegisteredRoute<L, E>
{
    async fn run(self, robot: &mut crate::CompetitionBot<L>) -> Result<(), E> {
        if let Some(handler) = self.handler {
            handler(robot).await
        } else {
            Ok(())
        }
    }

    fn allows_autonomous(self) -> bool {
        self.handler.is_some()
    }
}

/// Name an async robot function for autonomous selection or driver dispatch.
#[macro_export]
macro_rules! route {
    ($label:expr, $handler:path $(,)?) => {
        $crate::competition::RegisteredRoute::new($label, |robot| {
            ::std::boxed::Box::pin(async move {
                $crate::competition::IntoRouteResult::into_route_result($handler(robot).await)
            })
        })
    };
}

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

    fn menu(&mut self) -> &mut RouteMenu<Option<Self::Route>>;
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
        ready: impl Fn(&Self) -> bool,
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

/// The interface used to select a route and display competition status.
pub enum Ui {
    Brain(Display),
}

/// Brain selection and status for an assembled robot.
pub struct Autonomous<R: Copy + 'static, S = ()> {
    menu: RouteMenu<Option<R>>,
    sampler: S,
    started: Option<Instant>,
    driver_routes: DriverRoutes<R>,
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
        }
    }
}

impl<R: RouteSet, S> Autonomous<R, S> {
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
{
    fn record(robot: &mut Robot<D, C, T, X, P, Self>, sample: Sample) {
        if let Some(frame) = robot.autonomous.sampler.frame(sample) {
            S::collect(robot, frame, sample);
        }
    }

    fn status(robot: &mut Robot<D, C, T, X, P, Self>, status: &str) {
        robot.autonomous.status(status);
    }

    fn driver_output(robot: &mut Robot<D, C, T, X, P, Self>, elapsed: Duration) {
        robot.autonomous.sampler.driver_output(elapsed);
    }
}

/// Displayed during ordinary driver control.
pub const DRIVER_STATUS: &str = "Driver control";

/// Application hooks invoked by the programmed competition loop.
#[allow(async_fn_in_trait)]
pub trait Program<R, D: ProgramDrive> {
    type Error: std::fmt::Display;
    const CONTROL_PERIOD: Duration = CONTROL_PERIOD;
    const AUTON_TIMEOUT: Duration = Duration::from_secs(15);
    const MAX_CONTROL_GAP: Duration = Duration::from_millis(100);

    /// Translate a drive failure into the program's error type.
    fn drive_error(error: D::Error, stopping: bool) -> Self::Error;

    /// Driver callback. Read the current controller sample and elapsed time from
    /// the robot. Issue a drive command with `Robot::drive_driver`.
    async fn driver(robot: &mut R) -> Result<(), Self::Error>;

    /// Refresh lightweight inputs needed by a driver callback.
    fn before_driver(_robot: &mut R) {}

    /// Bit mask of beams blocked by the current mechanism pose.
    fn range_occlusion(_robot: &R) -> u8 {
        0
    }

    /// Service the same mechanism coordinator during driver and autonomous ticks.
    fn update_mechanisms(
        _robot: &mut R,
        _mode: CompetitionMode,
        _dt: Duration,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Stop custom actuators and clear pending requests. Ozton stops the drive separately.
    fn stop_mechanisms(_robot: &mut R) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Remove actuator output for a recoverable motion failure.
    fn pause_mechanisms(robot: &mut R) -> Result<(), Self::Error> {
        Self::stop_mechanisms(robot)
    }

    /// Release a finished driver's mechanism commands. The drive remains under joystick control.
    fn finish_driver_route(robot: &mut R) -> Result<(), Self::Error> {
        Self::stop_mechanisms(robot)
    }

    /// Optional tracking work runs after driver outputs and within the tick budget.
    fn poll_tracking(_robot: &mut R, _mode: CompetitionMode) {}
    fn mode_changed(_robot: &mut R, _mode: CompetitionMode) {}
    fn after_tick(_robot: &mut R, _mode: CompetitionMode, _tick: Tick) {}
    fn route_error(
        _robot: &mut R,
        _mode: CompetitionMode,
        _name: &'static str,
        _error: &Self::Error,
    ) -> Option<&'static str> {
        None
    }
}

impl<D, T, X, P, R, S> CompetitionRobot for Robot<D, Controller, T, X, P, Autonomous<R, S>>
where
    D: ProgramDrive,
    P: Program<Self, D>,
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
        self.autonomous.driver_routes.finish();
        stop_program(self)
    }

    fn set_autonomous_start(&mut self, started: Option<Instant>) {
        self.autonomous.started = started;
    }

    async fn driver_tick(&mut self, tick: Tick) {
        let input = self.controller.state().ok();
        let active = self.autonomous.driver_routes.active;
        if input.is_none() && self.autonomous.driver_routes.finish() {
            self.autonomous.status("Controller unavailable");
        }
        let result = while_in_mode(
            dispatch_driver(self, input, tick),
            CompetitionMode::Driver,
            Self::CONTROL_PERIOD,
        )
        .await;
        let Some(result) = result else {
            let _ = self.stop();
            return;
        };
        if let Err(error) = result {
            crate::diagnostic_event!("driver_tick_failed,error={error}");
            let stopped = self.stop();
            if let Some(route) = active {
                report_route_error::<D, _, P>(self, CompetitionMode::Driver, route.label(), &error);
                if let Err(stop) = stopped {
                    report_route_error::<D, _, P>(
                        self,
                        CompetitionMode::Driver,
                        route.label(),
                        &stop,
                    );
                }
            }
        }
    }

    fn driver_route_active(&self) -> bool {
        self.autonomous.driver_routes.active.is_some()
    }

    fn cancel_driver_route(&mut self) -> Result<(), Self::Error> {
        if self.autonomous.driver_routes.finish() {
            P::finish_driver_route(self)
        } else {
            Ok(())
        }
    }

    async fn driver_action(&mut self) {
        let Some(route) = self.autonomous.driver_routes.start() else {
            return;
        };
        let result = while_in_mode(
            route.run(self),
            CompetitionMode::Driver,
            Self::CONTROL_PERIOD,
        )
        .await;
        let was_active = self.driver_route_active();
        let stopped = self.cancel_driver_route();
        if was_active && let Some(Err(error)) = result {
            report_route_error::<D, _, P>(self, CompetitionMode::Driver, route.label(), &error);
        }
        if let Err(error) = stopped {
            let _ = self.stop();
            report_route_error::<D, _, P>(self, CompetitionMode::Driver, route.label(), &error);
        }
    }

    fn mode_changed(&mut self, mode: CompetitionMode) {
        P::mode_changed(self, mode);
    }

    fn after_tick(&mut self, mode: CompetitionMode, tick: Tick) {
        if tracking_poll_due(mode, tick.work(), Self::CONTROL_PERIOD) {
            P::poll_tracking(self, mode);
        }
        P::after_tick(self, mode, tick);
        self.record_diagnostics(Sample {
            mode,
            dt: tick.elapsed,
            motion: None,
        });
    }

    fn route_failed(&mut self, route: Self::Route, error: Self::Error) {
        report_route_error::<D, _, P>(self, CompetitionMode::Autonomous, route.label(), &error);
    }
}

fn report_route_error<D, R, P>(
    robot: &mut R,
    mode: CompetitionMode,
    name: &'static str,
    error: &P::Error,
) where
    D: ProgramDrive,
    P: Program<R, D>,
    R: CompetitionRobot,
{
    crate::diagnostic_event!("route={name},mode={mode:?},error={error}");
    if let Some(status) = P::route_error(robot, mode, name, error) {
        robot.menu().status(status);
    } else {
        robot.menu().status(&error.to_string());
    }
}

fn stop_program<D, C, T, X, P, A>(robot: &mut Robot<D, C, T, X, P, A>) -> Result<(), P::Error>
where
    D: ProgramDrive,
    P: Program<Robot<D, C, T, X, P, A>, D>,
{
    robot.driver_input = None;
    let drive = robot
        .drivetrain
        .stop()
        .map_err(|error| P::drive_error(error, true));
    drive.and(P::stop_mechanisms(robot))
}

async fn dispatch_driver<D, C, T, X, P, A>(
    robot: &mut Robot<D, C, T, X, P, A>,
    input: Option<ControllerState>,
    tick: Tick,
) -> Result<(), P::Error>
where
    D: ProgramDrive,
    P: Program<Robot<D, C, T, X, P, A>, D>,
    A: Telemetry<Robot<D, C, T, X, P, A>>,
{
    let Some(input) = input else {
        return stop_program(robot);
    };
    robot.driver_input = Some(input);
    robot.driver_elapsed = tick.elapsed;
    robot.driver_wrote = false;
    P::before_driver(robot);
    P::driver(robot).await?;
    if !robot.driver_wrote {
        robot
            .drivetrain
            .stop()
            .map_err(|error| P::drive_error(error, true))?;
    }
    robot.mark_driver_output(tick);
    P::update_mechanisms(robot, CompetitionMode::Driver, tick.elapsed)
}

fn tracking_poll_due(mode: CompetitionMode, work: Duration, period: Duration) -> bool {
    match mode {
        CompetitionMode::Disabled => false,
        CompetitionMode::Driver => work < period,
        CompetitionMode::Autonomous => true,
    }
}

pub(crate) fn autonomous_ui<R: RouteSet>(
    ui: Ui,
    routes: impl IntoIterator<Item = R>,
) -> Autonomous<R> {
    let Ui::Brain(display) = ui;
    Autonomous {
        menu: RouteMenu::with_choices(
            display,
            None,
            routes.into_iter().map(|route| (route.label(), Some(route))),
        ),
        sampler: (),
        started: None,
        driver_routes: DriverRoutes {
            queued: None,
            active: None,
        },
    }
}

/// Run the common mode loop, cancelling any route when autonomous ends.
pub(crate) async fn compete<R: CompetitionRobot>(mut robot: R) -> ! {
    let _sd_writer = crate::diagnostics::start_sd_logging();
    crate::diagnostic_event!("boot,control_period_us={}", R::CONTROL_PERIOD.as_micros());
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
            crate::diagnostic_event!("competition_mode={mode:?}");
            robot.menu().poll(disabled);
            robot.mode_changed(mode);
            if mode == CompetitionMode::Driver {
                robot.menu().status(DRIVER_STATUS);
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
                let status = if robot.selected_autonomous().is_some() {
                    "Ready for autonomous"
                } else {
                    "Autonomous disabled"
                };
                robot.menu().status(status);
            }
            CompetitionMode::Driver => robot.driver_tick(tick).await,
            CompetitionMode::Autonomous if entering => {
                if let Some(route) = robot.selected_autonomous() {
                    robot
                        .menu()
                        .status(&format!("Autonomous: {}", route.label()));
                    crate::diagnostic_event!("auton_route={},state=start", route.label());
                    let result = match robot.stop() {
                        Ok(()) => {
                            robot.set_autonomous_start(Some(Instant::now()));
                            while_in_mode(
                                route.run(&mut robot),
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
                            crate::diagnostic_event!(
                                "auton_route={},state=complete",
                                route.label()
                            );
                        }
                        (None, Ok(())) => {
                            robot.menu().status("Autonomous cancelled");
                            crate::diagnostic_event!(
                                "auton_route={},state=cancelled",
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
        if mode == CompetitionMode::Driver && competition::mode() == mode {
            let action_started = Instant::now();
            robot.driver_action().await;
            if action_started.elapsed() >= R::CONTROL_PERIOD {
                last_tick = Instant::now();
                if competition::mode() == mode {
                    sleep(R::CONTROL_PERIOD).await;
                }
                continue;
            }
        }
        sleep(control_sleep(R::CONTROL_PERIOD, tick.work())).await;
    }
}

async fn while_in_mode<T>(
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

fn control_sleep(period: Duration, work: Duration) -> Duration {
    period.saturating_sub(work).max(Duration::from_millis(1))
}

#[cfg(test)]
mod tests {
    use super::*;

    struct FakeDrive {
        writes: Vec<&'static str>,
        fail_stop: bool,
    }

    impl ozton_drivetrain::model::DrivetrainModel for FakeDrive {
        type Error = &'static str;
    }

    impl ProgramDrive for FakeDrive {
        type DriverCommand = bool;
        type AutonCommand = i32;

        fn apply_driver(&mut self, command: bool, _: Duration) -> Result<(), Self::Error> {
            assert!(command);
            self.writes.push("drive");
            Ok(())
        }

        fn apply_auton(&mut self, command: i32, _: Duration) -> Result<(), Self::Error> {
            assert_eq!(command, 7);
            self.writes.push("auton");
            Ok(())
        }

        fn stop(&mut self) -> Result<(), Self::Error> {
            self.writes.push("stop drive");
            if self.fail_stop { Err("port") } else { Ok(()) }
        }
    }

    struct FakeProgram;
    struct NoTelemetry;
    type TestRobot = Robot<FakeDrive, (), (), (), FakeProgram, NoTelemetry>;

    impl Telemetry<TestRobot> for NoTelemetry {
        fn record(_: &mut TestRobot, _: Sample) {}
        fn status(_: &mut TestRobot, _: &str) {}
    }

    impl Program<TestRobot, FakeDrive> for FakeProgram {
        type Error = &'static str;

        fn drive_error(error: &'static str, _: bool) -> Self::Error {
            error
        }

        async fn driver(robot: &mut TestRobot) -> Result<(), Self::Error> {
            assert_eq!(robot.driver_input(), Some(ControllerState::default()));
            robot.drivetrain.writes.push("driver");
            robot.drive_driver(true)
        }

        fn update_mechanisms(
            robot: &mut TestRobot,
            mode: CompetitionMode,
            _: Duration,
        ) -> Result<(), Self::Error> {
            assert_eq!(mode, CompetitionMode::Driver);
            robot.drivetrain.writes.push("mechanisms");
            Ok(())
        }

        fn stop_mechanisms(robot: &mut TestRobot) -> Result<(), Self::Error> {
            robot.drivetrain.writes.push("stop mechanisms");
            Ok(())
        }
    }

    #[test]
    fn queued_driver_route_starts_once_and_clears_on_cancellation() {
        let mut routes = DriverRoutes {
            queued: None,
            active: None,
        };
        routes.queue("score");
        assert_eq!(routes.start(), Some("score"));
        routes.queue("other");
        assert_eq!(routes.start(), None);
        assert_eq!(routes.active, Some("score"));
        assert!(routes.finish());
        assert!(!routes.finish());
        assert_eq!(routes.start(), None);

        routes.queue("score");
        assert!(!routes.finish());
        assert_eq!(routes.start(), None);
    }

    #[test]
    fn driver_dispatch_and_disconnect_stop_use_the_installed_drive() {
        let mut robot = TestRobot {
            drivetrain: FakeDrive {
                writes: Vec::new(),
                fail_stop: false,
            },
            controller: (),
            tracking: (),
            mechanisms: (),
            program: FakeProgram,
            autonomous: NoTelemetry,
            driver_input: None,
            driver_elapsed: Duration::ZERO,
            driver_wrote: false,
        };
        let tick = Tick {
            started: Instant::now(),
            elapsed: CONTROL_PERIOD,
        };
        assert_eq!(
            pin!(dispatch_driver(
                &mut robot,
                Some(ControllerState::default()),
                tick
            ))
            .poll(&mut core::task::Context::from_waker(
                core::task::Waker::noop()
            )),
            Poll::Ready(Ok(()))
        );
        assert_eq!(robot.drivetrain.apply_auton(7, CONTROL_PERIOD), Ok(()));
        robot.drivetrain.fail_stop = true;
        assert_eq!(
            pin!(dispatch_driver(&mut robot, None, tick)).poll(
                &mut core::task::Context::from_waker(core::task::Waker::noop())
            ),
            Poll::Ready(Err("port"))
        );
        assert_eq!(robot.driver_input(), None);
        assert_eq!(
            robot.drivetrain.writes,
            [
                "driver",
                "drive",
                "mechanisms",
                "auton",
                "stop drive",
                "stop mechanisms"
            ]
        );
    }

    #[test]
    fn overrunning_driver_tick_skips_optional_tracking() {
        assert!(tracking_poll_due(
            CompetitionMode::Driver,
            Duration::ZERO,
            CONTROL_PERIOD
        ));
        assert!(!tracking_poll_due(
            CompetitionMode::Driver,
            CONTROL_PERIOD,
            CONTROL_PERIOD
        ));
        assert!(!tracking_poll_due(
            CompetitionMode::Disabled,
            Duration::ZERO,
            CONTROL_PERIOD
        ));
        assert!(tracking_poll_due(
            CompetitionMode::Autonomous,
            CONTROL_PERIOD,
            CONTROL_PERIOD
        ));
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
