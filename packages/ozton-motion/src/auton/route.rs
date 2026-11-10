//! Register named route functions and prepare embedded assets at startup.

use std::{convert::Infallible, future::Future, pin::Pin, time::Duration};

use ozton_math::{
    geometry::Pose,
    path::{Limits, Path, PathError, follower::Follower, jerryio},
};
use ozton_runtime::clock::CONTROL_PERIOD;
use vexide::{display::Display, prelude::Controller};

use super::{
    Alignment, AutonError, AutonRobot, Calibration, Command, Mechanisms, PathMarker, RangeId,
    Robot, Runner,
    calibration::{CalibrationRun, PreparedCalibration},
    chassis::ChassisError,
    competition::{CompetitionApp, compete_app},
};

pub type RouteResult<T = ()> = Result<T, AutonError<ChassisError>>;
type RouteFuture<'a> = Pin<Box<dyn Future<Output = RouteResult> + 'a>>;

trait RunRoute<M: Mechanisms> {
    fn run<'a>(&'a self, auton: Auton<'a, M>) -> RouteFuture<'a>;
}

impl<M: Mechanisms, F: for<'a> AsyncFn(Auton<'a, M>) -> RouteResult> RunRoute<M> for F {
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
pub struct Route<M: Mechanisms> {
    name: &'static str,
    start: Pose,
    asset: Asset,
    run: Box<dyn RunRoute<M>>,
}

impl<M: Mechanisms> Route<M> {
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
    pub distance: f64,
    pub heading: f64,
    pub maximum_speed: f64,
    pub rear_facing: bool,
}

impl AlignTarget {
    pub const fn front(distance: f64, heading: f64) -> Self {
        Self {
            distance,
            heading,
            maximum_speed: 0.2,
            rear_facing: false,
        }
    }
}

struct Prepared<M: Mechanisms> {
    name: &'static str,
    start: Pose,
    path: Option<Path>,
    run: Box<dyn RunRoute<M>>,
}

/// Route-local commands with a shared deadline and a prepared primary path.
pub struct Auton<'a, M: Mechanisms> {
    runner: Runner<'a, Robot<M>>,
    path: Option<&'a Path>,
}

impl<M: Mechanisms> Auton<'_, M> {
    pub fn mechanisms(&mut self) -> &mut M {
        &mut self.runner.robot().mechanisms
    }
    pub fn range(&mut self, name: &str) -> RouteResult<RangeId> {
        self.runner
            .robot()
            .range(name)
            .ok_or(AutonError::Configuration)
    }

    pub async fn follow(&mut self, timeout: Duration) -> RouteResult {
        self.follow_with(
            Follower::PurePursuit { lookahead: 0.16 },
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
        sensor: RangeId,
        target: AlignTarget,
        timeout: Duration,
    ) -> RouteResult<bool> {
        let maximum_speed = self.runner.robot().drive.drive_options.maximum_wheel_speed;
        if !target.maximum_speed.is_finite() {
            return Err(AutonError::Configuration);
        }
        self.runner
            .try_align(
                Alignment {
                    sensor: sensor.0,
                    distance: target.distance,
                    heading: target.heading,
                    maximum_speed: target.maximum_speed.min(maximum_speed),
                    direction: if target.rear_facing { -1.0 } else { 1.0 },
                },
                timeout,
            )
            .await
    }

    pub async fn run(&mut self, command: Command<'_, Robot<M>>, timeout: Duration) -> RouteResult {
        self.runner.run(command, timeout).await
    }
}

/// Prepare all assets before competition starts and show the named routes on the Brain.
pub async fn compete_routes<M: Mechanisms + 'static>(
    robot: Robot<M>,
    display: Display,
    controller: Controller,
    routes: impl IntoIterator<Item = Route<M>>,
) -> Result<Infallible, RegistrationError> {
    compete_with_calibration(robot, display, controller, routes, []).await
}

/// Add supervised driver-mode calibration entries to the Brain selector.
pub async fn compete_with_calibration<M: Mechanisms + 'static>(
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

struct Registered<M: Mechanisms> {
    robot: Robot<M>,
    controller: Controller,
    prepared: Vec<Prepared<M>>,
    trials: Vec<PreparedCalibration>,
    sampling: Option<CalibrationRun>,
}

impl<M: Mechanisms> CompetitionApp for Registered<M> {
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
        let settings = self.robot.drive.settings;
        let options = self.robot.drive.drive_options;
        let runner = Runner::new(
            &mut self.robot,
            options,
            CONTROL_PERIOD,
            settings.max_control_gap,
            settings.route_timeout,
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
