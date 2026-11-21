//! Register named route functions and prepare embedded assets at startup.

use std::{convert::Infallible, future::Future, pin::Pin, time::Duration};

use ozton_math::{
    geometry::Pose,
    path::{Limits, Path, PathError, follower::Follower, jerryio},
    uom::{
        ConstZero,
        si::{
            f64::{Angle, Length, Velocity},
            length::meter,
            velocity::meter_per_second,
        },
    },
};
use ozton_runtime::{SubsystemList, clock::CONTROL_PERIOD};
use vexide::{display::Display, prelude::Controller, smart::PortError};

use super::{
    Alignment, AutonError, AutonRobot, Calibration, Command, PathMarker, Robot, Runner, StockTick,
    calibration::{CalibrationRun, PreparedCalibration},
    chassis::ChassisError,
    competition::{CompetitionApp, compete_app},
};

pub type RouteResult<T = ()> = Result<T, AutonError<ChassisError>>;
type RouteFuture<'a> = Pin<Box<dyn Future<Output = RouteResult> + 'a>>;

trait RunRoute<M: for<'tick> SubsystemList<StockTick<'tick>, PortError>> {
    fn run<'a>(&'a self, auton: Auton<'a, M>) -> RouteFuture<'a>;
}

impl<
    M: for<'tick> SubsystemList<StockTick<'tick>, PortError>,
    F: for<'a> AsyncFn(Auton<'a, M>) -> RouteResult,
> RunRoute<M> for F
{
    fn run<'a>(&'a self, auton: Auton<'a, M>) -> RouteFuture<'a> {
        Box::pin(self(auton))
    }
}

enum Asset {
    None,
    JerryIo(&'static [u8]),
    Chain(&'static [&'static [u8]]),
}

/// A menu label, starting pose, optional path asset and ordinary Rust route function.
pub struct Route<M: for<'tick> SubsystemList<StockTick<'tick>, PortError>> {
    name: &'static str,
    start: Pose,
    asset: Asset,
    run: Box<dyn RunRoute<M>>,
}

impl<M: for<'tick> SubsystemList<StockTick<'tick>, PortError>> Route<M> {
    pub fn new(
        name: &'static str,
        start: Pose,
        run: impl for<'a> AsyncFn(Auton<'a, M>) -> RouteResult + 'static,
    ) -> Self {
        Self {
            name,
            start,
            asset: Asset::None,
            run: Box::new(run),
        }
    }

    pub fn jerryio(
        name: &'static str,
        start: Pose,
        bytes: &'static [u8],
        run: impl for<'a> AsyncFn(Auton<'a, M>) -> RouteResult + 'static,
    ) -> Self {
        Self {
            asset: Asset::JerryIo(bytes),
            ..Self::new(name, start, run)
        }
    }

    /// Join aligned, same-direction exports and profile them as one motion.
    pub fn jerryio_chain(
        name: &'static str,
        start: Pose,
        segments: &'static [&'static [u8]],
        run: impl for<'a> AsyncFn(Auton<'a, M>) -> RouteResult + 'static,
    ) -> Self {
        Self {
            asset: Asset::Chain(segments),
            ..Self::new(name, start, run)
        }
    }

    fn prepare(self, limits: Limits) -> Result<Prepared<M>, RegistrationError> {
        let error = |problem| RegistrationError {
            route: self.name,
            problem,
        };
        if self.name.trim().is_empty() || !self.start.is_finite() {
            return Err(error(RegistrationProblem::InvalidRoute));
        }
        let points = match self.asset {
            Asset::None => None,
            Asset::JerryIo(bytes) => Some(
                jerryio::parse(bytes)
                    .map_err(|problem| error(RegistrationProblem::Import(problem)))?,
            ),
            Asset::Chain(segments) => Some(
                jerryio::parse_chain(segments)
                    .map_err(|problem| error(RegistrationProblem::Chain(problem)))?,
            ),
        };
        let path = points
            .map(|points| {
                Path::new(&points, limits)
                    .map_err(|problem| error(RegistrationProblem::Path(problem)))
            })
            .transpose()?;
        Ok(Prepared {
            name: self.name,
            start: self.start,
            path,
            run: self.run,
        })
    }
}

#[derive(Debug)]
pub struct RegistrationError {
    pub route: &'static str,
    pub problem: RegistrationProblem,
}

#[derive(Debug)]
pub enum RegistrationProblem {
    InvalidRoute,
    DuplicateName,
    Import(jerryio::ImportError),
    Chain(jerryio::ChainError),
    Path(PathError),
    InvalidCalibration,
}

/// Relative alignment target. The selected sensor supplies the distance.
#[derive(Debug, Clone, Copy)]
pub struct AlignTarget {
    pub distance: Length,
    pub heading: Angle,
    pub maximum_speed: Velocity,
    pub rear_facing: bool,
}

impl AlignTarget {
    pub const fn front(distance: Length, heading: Angle) -> Self {
        Self {
            distance,
            heading,
            maximum_speed: Velocity {
                value: 0.2,
                ..Velocity::ZERO
            },
            rear_facing: false,
        }
    }
}

struct Prepared<M: for<'tick> SubsystemList<StockTick<'tick>, PortError>> {
    name: &'static str,
    start: Pose,
    path: Option<Path>,
    run: Box<dyn RunRoute<M>>,
}

/// Route-local commands with a shared deadline and a prepared primary path.
pub struct Auton<'a, M: for<'tick> SubsystemList<StockTick<'tick>, PortError>> {
    runner: Runner<'a, Robot<M>>,
    path: Option<&'a Path>,
}

impl<M: for<'tick> SubsystemList<StockTick<'tick>, PortError>> Auton<'_, M> {
    pub fn subsystems(&mut self) -> &mut M {
        &mut self.runner.robot().subsystems
    }
    pub async fn follow(&mut self, timeout: Duration) -> RouteResult {
        self.follow_with(
            Follower::PurePursuit {
                lookahead: Length::new::<meter>(0.16),
            },
            false,
            &[],
            timeout,
        )
        .await
    }

    pub async fn follow_with(
        &mut self,
        follower: Follower,
        reverse: bool,
        markers: &[PathMarker<Robot<M>>],
        timeout: Duration,
    ) -> RouteResult {
        let path = self.path.ok_or(AutonError::Configuration)?;
        self.run(
            Command::FollowMarked {
                path,
                follower,
                reverse,
                markers,
            },
            timeout,
        )
        .await
    }

    pub async fn wait_until(
        &mut self,
        ready: fn(&Robot<M>) -> bool,
        timeout: Duration,
    ) -> RouteResult {
        self.run(Command::WaitUntil(ready), timeout).await
    }

    /// Try a relative range approach. A clean timeout leaves the route active.
    pub async fn try_align(
        &mut self,
        sensor: usize,
        target: AlignTarget,
        timeout: Duration,
    ) -> RouteResult<bool> {
        let maximum_speed = self.runner.robot().drive.drive_options.maximum_wheel_speed;
        if sensor >= self.runner.robot().drive.range_count()
            || !target.maximum_speed.value.is_finite()
        {
            return Err(AutonError::Configuration);
        }
        self.runner
            .try_align(
                Alignment {
                    sensor,
                    distance: target.distance,
                    heading: target.heading,
                    maximum_speed: Velocity::new::<meter_per_second>(
                        target.maximum_speed.value.min(maximum_speed.value),
                    ),
                    facing: if target.rear_facing {
                        super::RangeFacing::Rear
                    } else {
                        super::RangeFacing::Front
                    },
                },
                timeout,
            )
            .await
    }

    pub async fn drive_to_point(
        &mut self,
        target: [Length; 2],
        maximum_speed: Velocity,
        reverse: bool,
        timeout: Duration,
    ) -> RouteResult {
        self.runner
            .drive_to_point(target, maximum_speed, reverse, timeout)
            .await
    }
    pub async fn drive_distance(
        &mut self,
        meters: Length,
        heading: Angle,
        maximum_speed: Velocity,
        timeout: Duration,
    ) -> RouteResult {
        self.runner
            .drive_distance(meters, heading, maximum_speed, timeout)
            .await
    }
    pub async fn turn_to(&mut self, heading: Angle, timeout: Duration) -> RouteResult {
        self.runner.turn_to(heading, timeout).await
    }
    pub async fn wait(&mut self, duration: Duration, timeout: Duration) -> RouteResult {
        self.runner.wait(duration, timeout).await
    }
    pub async fn try_run(
        &mut self,
        command: Command<'_, Robot<M>>,
        timeout: Duration,
    ) -> RouteResult<bool> {
        self.runner.try_run(command, timeout).await
    }
    pub async fn run(&mut self, command: Command<'_, Robot<M>>, timeout: Duration) -> RouteResult {
        self.runner.run(command, timeout).await
    }
}

impl<A, M: super::MechanismActions> Auton<'_, (A, M)>
where
    (A, M): for<'tick> SubsystemList<StockTick<'tick>, PortError>,
{
    pub async fn perform(
        &mut self,
        target: M::Target,
        action_for: Duration,
        timeout: Duration,
    ) -> RouteResult {
        self.runner.perform(target, action_for, timeout).await
    }
}

/// Prepare all assets before competition starts and show the named routes on the Brain.
pub async fn compete_routes<M: for<'tick> SubsystemList<StockTick<'tick>, PortError> + 'static>(
    robot: Robot<M>,
    display: Display,
    controller: Controller,
    routes: impl IntoIterator<Item = Route<M>>,
) -> Result<Infallible, RegistrationError> {
    compete_with_calibration(robot, display, controller, routes, []).await
}

/// Add supervised driver-mode calibration entries to the Brain selector.
pub async fn compete_with_calibration<
    M: for<'tick> SubsystemList<StockTick<'tick>, PortError> + 'static,
>(
    robot: Robot<M>,
    display: Display,
    controller: Controller,
    routes: impl IntoIterator<Item = Route<M>>,
    calibration: impl IntoIterator<Item = Calibration>,
) -> Result<Infallible, RegistrationError> {
    let mut prepared: Vec<Prepared<M>> = Vec::new();
    for route in routes {
        if route.name == "Disable" || prepared.iter().any(|other| other.name == route.name) {
            return Err(RegistrationError {
                route: route.name,
                problem: RegistrationProblem::DuplicateName,
            });
        }
        prepared.push(route.prepare(robot.drive.settings.path_limits())?);
    }
    let mut choices: Vec<_> = prepared
        .iter()
        .enumerate()
        .map(|(index, route)| (route.name, Menu::Route(index)))
        .collect();
    let mut trials: Vec<PreparedCalibration> = Vec::new();
    for trial in calibration {
        if trial.name == "Disable" || choices.iter().any(|(name, _)| *name == trial.name) {
            return Err(RegistrationError {
                route: trial.name,
                problem: RegistrationProblem::DuplicateName,
            });
        }
        if robot.log_file() == Some(trial.file) || trials.iter().any(|case| case.file == trial.file)
        {
            return Err(RegistrationError {
                route: trial.name,
                problem: RegistrationProblem::InvalidCalibration,
            });
        }
        let case = PreparedCalibration::new(trial, &robot.drive).ok_or(RegistrationError {
            route: trial.name,
            problem: RegistrationProblem::InvalidCalibration,
        })?;
        choices.push((case.name, Menu::Calibration(trials.len())));
        trials.push(case);
    }
    let sampling = (!trials.is_empty()).then(CalibrationRun::new);
    compete_app(
        Registered {
            robot,
            controller,
            prepared,
            trials,
            sampling,
        },
        display,
        &choices,
    )
    .await
}

#[derive(Clone, Copy)]
enum Menu {
    Route(usize),
    Calibration(usize),
}

struct Registered<M: for<'tick> SubsystemList<StockTick<'tick>, PortError>> {
    robot: Robot<M>,
    controller: Controller,
    prepared: Vec<Prepared<M>>,
    trials: Vec<PreparedCalibration>,
    sampling: Option<CalibrationRun>,
}

impl<M: for<'tick> SubsystemList<StockTick<'tick>, PortError>> CompetitionApp for Registered<M> {
    type Route = Menu;
    type Error = ChassisError;

    fn stop(&mut self) {
        let _ = self.robot.stop();
        if let Some(sampling) = &mut self.sampling {
            sampling.cancel();
        }
    }

    fn driver(&mut self, selected: Option<Menu>, dt: Duration) -> Option<&'static str> {
        if let Ok(state) = self.controller.state() {
            if let Some(Menu::Calibration(index)) = selected
                && let (Some(trial), Some(sampling)) =
                    (self.trials.get(index), self.sampling.as_mut())
            {
                return sampling.tick(trial, &mut self.robot.drive, &state, dt);
            }
            self.robot.driver_tick(&state, dt);
        } else {
            self.stop();
        }
        None
    }

    fn allows_autonomous(&self, route: Menu) -> bool {
        matches!(route, Menu::Route(_))
    }

    async fn run(&mut self, route: Menu) -> RouteResult {
        let Menu::Route(route) = route else {
            return Ok(());
        };
        let route = self.prepared.get(route).ok_or(AutonError::Configuration)?;
        self.robot
            .set_pose(route.start)
            .map_err(AutonError::Robot)?;
        let max_control_gap = self.robot.drive.settings.max_control_gap;
        let route_timeout = self.robot.drive.settings.route_timeout;
        let options = self.robot.drive.drive_options;
        let runner = Runner::new(
            &mut self.robot,
            options,
            CONTROL_PERIOD,
            max_control_gap,
            route_timeout,
        )?;
        route
            .run
            .run(Auton {
                runner,
                path: route.path.as_ref(),
            })
            .await
    }

    fn disabled(&mut self) -> Option<&'static str> {
        let telemetry = self.robot.save_log();
        self.sampling
            .as_mut()
            .and_then(CalibrationRun::save)
            .or(telemetry)
    }
    fn status(&self) -> Option<&'static str> {
        Some(self.robot.drive.chassis.tracking_status())
    }
}
