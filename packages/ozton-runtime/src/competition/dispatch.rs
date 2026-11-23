//! Driver ticks and fault handling on an assembled robot.
use super::*;

impl<D, T, X, P, R, S> CompetitionRobot for Robot<D, Controller, T, X, P, Autonomous<R, S>>
where
    D: ProgramDrive,
    P: Program<Self, D>,
    R: Routes<Self, P::Error>,
    Autonomous<R, S>: Telemetry<Self>,
    X: Mechanism<P::Error>,
{
    type Error = P::Error;
    type Route = R;
    const CONTROL_PERIOD: Duration = P::CONTROL_PERIOD;

    fn menu(&mut self) -> &mut dyn RouteSelector<Option<R>> {
        self.autonomous.menu.as_mut()
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        stop_program(self)
    }

    fn set_autonomous_start(&mut self, started: Option<Instant>) {
        self.autonomous.started = started;
    }

    async fn driver_tick(&mut self, tick: Tick) {
        let result = while_in_mode(
            dispatch_driver(self, tick),
            CompetitionMode::Driver,
            Self::CONTROL_PERIOD,
        )
        .await;
        match result {
            None => {
                let _ = self.stop();
            }
            Some(Err(error)) => {
                if let Err(_stop) = self.stop() {
                    crate::diagnostic_event!("driver_stop_failed,error={_stop}");
                }
                self.autonomous.report_driver_fault(Some(error.to_string()));
            }
            Some(Ok(())) => {
                let output_fault = self
                    .drivetrain
                    .take_driver_fault()
                    .map(|error| P::drive_error(error, false).to_string());
                let mechanism_fault = self
                    .update_mechanisms(CompetitionMode::Driver, tick.elapsed)
                    .err()
                    .map(|error| {
                        if let Err(_stop) = self.stop_mechanisms() {
                            crate::diagnostic_event!("driver_mechanism_stop_failed,error={_stop}");
                        }
                        error.to_string()
                    });
                self.autonomous
                    .report_driver_fault(mechanism_fault.or(output_fault));
            }
        }
    }

    fn mode_changed(&mut self, mode: CompetitionMode) {
        self.autonomous.last_driver_fault = None;
        self.drivetrain.competition_mode_changed(mode);
        P::mode_changed(self, mode);
    }

    fn after_tick(&mut self, mode: CompetitionMode, tick: Tick) {
        if tracking_poll_due(mode, tick.work(), Self::CONTROL_PERIOD) {
            self.drivetrain.poll_competition_tracking(mode);
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

pub(super) fn stop_program<D, C, T, X, P, A>(
    robot: &mut Robot<D, C, T, X, P, A>,
) -> Result<(), P::Error>
where
    D: ProgramDrive,
    P: Program<Robot<D, C, T, X, P, A>, D>,
    X: Mechanism<P::Error>,
{
    let drive = robot
        .drivetrain
        .stop()
        .map_err(|error| P::drive_error(error, true));
    drive.and(robot.stop_mechanisms())
}

pub(super) async fn dispatch_driver<D, C, T, X, P, A>(
    robot: &mut Robot<D, C, T, X, P, A>,
    tick: Tick,
) -> Result<(), P::Error>
where
    D: ProgramDrive,
    P: Program<Robot<D, C, T, X, P, A>, D>,
    A: Telemetry<Robot<D, C, T, X, P, A>>,
    X: Mechanism<P::Error>,
{
    robot.driver_elapsed = tick.elapsed;
    robot.driver_wrote = false;
    robot.drivetrain.take_driver_output();
    robot.drivetrain.take_driver_fault();
    P::before_driver(robot);
    P::driver(robot).await?;
    if !robot.drivetrain.take_driver_output() && !robot.driver_wrote {
        robot
            .drivetrain
            .stop()
            .map_err(|error| P::drive_error(error, true))?;
    }
    robot.mark_driver_output(tick);
    Ok(())
}

pub(super) fn tracking_poll_due(mode: CompetitionMode, work: Duration, period: Duration) -> bool {
    match mode {
        CompetitionMode::Disabled => false,
        CompetitionMode::Driver => work < period,
        CompetitionMode::Autonomous => true,
    }
}
