//! Competition dispatch for a robot assembled by the application.

use std::{fmt::Display, time::Duration};

use super::{
    Autonomous, Compete, CompetitionMode, CompetitionRobot, DriverLoop, ProgramDrive, RobotRoute,
    RouteSelector, Routes, Tick, Ui, autonomous_ui, session, while_in_mode,
};
use crate::{
    clock::CONTROL_PERIOD,
    diagnostics::{ComponentReport, Frame, SD_SAMPLE_INTERVAL, Sample, Sampler},
    localization::LocalizationWorker,
};

/// Hardware and behavior supplied by an application to the competition runtime.
#[allow(async_fn_in_trait)]
pub trait OztonCompete: Sized + 'static {
    type Error: Display + From<<Self::Drivetrain as ProgramDrive>::Error> + 'static;
    type Drivetrain: ProgramDrive + ComponentReport;

    const CONTROL_PERIOD: Duration = CONTROL_PERIOD;

    fn drivetrain(&mut self) -> &mut Self::Drivetrain;
    /// Service application-owned mechanisms after driver commands.
    fn update_mechanisms(
        &mut self,
        _mode: CompetitionMode,
        _dt: Duration,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Cancel mechanism work and stop its outputs on a mode transition or fault.
    fn stop_mechanisms(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn report_mechanisms(&self, _frame: Frame) {}

    const ROUTES: &'static [RobotRoute<Self, Self::Error>] = &[];

    /// Create the localization worker once, after hardware initialization.
    fn start_localization(&self) -> Option<LocalizationWorker> {
        None
    }

    /// Run for one driver activation, yielding via `driver.finish_iteration(self)`.
    async fn driver(&mut self, driver: &mut DriverLoop<'_, Self>) -> Result<(), Self::Error>;
}

/// Runs competition lifecycle and owns its selector and telemetry.
pub struct Runtime<R: OztonCompete> {
    robot: R,
    autonomous: Autonomous<RobotRoute<R, R::Error>, Sampler<()>>,
    localization: Option<LocalizationWorker>,
}

impl<R: OztonCompete> Runtime<R> {
    /// Create a runtime with autonomous selection disabled until a UI is supplied.
    pub fn new(robot: R) -> Self {
        Self {
            robot,
            autonomous: autonomous_ui(Ui::custom(|_| DisabledSelector), [])
                .with_diagnostics(SD_SAMPLE_INTERVAL),
            localization: None,
        }
    }

    /// Give the runtime a selector for the robot's registered routes.
    #[must_use]
    pub fn with_ui(mut self, ui: Ui<Option<RobotRoute<R, R::Error>>>) -> Self {
        self.autonomous =
            autonomous_ui(ui, R::ROUTES.iter().copied()).with_diagnostics(SD_SAMPLE_INTERVAL);
        self
    }

    pub async fn compete(mut self) -> ! {
        self.localization = self.robot.start_localization();
        session::compete(self).await
    }
}

struct DisabledSelector;

impl<A> RouteSelector<Option<A>> for DisabledSelector {
    fn action(&self) -> Option<A> {
        None
    }

    fn poll(&mut self, _: bool) -> Option<Option<A>> {
        None
    }

    fn status(&mut self, _: &str) {}
}

impl<R: OztonCompete> Routes<Runtime<R>, R::Error> for RobotRoute<R, R::Error> {
    async fn run(self, runtime: &mut Runtime<R>) -> Result<(), R::Error> {
        Routes::run(self, &mut runtime.robot).await
    }

    fn allows_autonomous(self) -> bool {
        <Self as Routes<R, R::Error>>::allows_autonomous(self)
    }
}

impl<R: OztonCompete> CompetitionRobot for Runtime<R> {
    type Error = R::Error;
    type Route = RobotRoute<R, R::Error>;
    const CONTROL_PERIOD: Duration = R::CONTROL_PERIOD;

    fn menu(&mut self) -> &mut dyn RouteSelector<Option<Self::Route>> {
        self.autonomous.menu.as_mut()
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        let drive = self.robot.drivetrain();
        drive.take_driver_output();
        let drive = drive.stop().map_err(R::Error::from);
        drive.and(self.robot.stop_mechanisms())
    }

    async fn driver_tick(&mut self, _: Tick) {}

    async fn driver_mode(&mut self) -> Result<(), Self::Error> {
        let mut driver = DriverLoop::new(&mut self.autonomous);
        while_in_mode(
            self.robot.driver(&mut driver),
            CompetitionMode::Driver,
            R::CONTROL_PERIOD,
        )
        .await
        .unwrap_or(Ok(()))
    }

    fn mode_changed(&mut self, mode: CompetitionMode) {
        if let Some(localization) = &self.localization {
            localization.mode_changed(mode);
        }
    }

    fn after_tick(&mut self, mode: CompetitionMode, tick: Tick) {
        record(&mut self.robot, &mut self.autonomous.sampler, mode, tick);
    }
}

impl<R: OztonCompete> Compete for Runtime<R> {
    async fn connected(&mut self) {
        session::CompetitionSession(self).stop();
    }

    async fn disconnected(&mut self) {
        session::CompetitionSession(self).stop();
    }

    async fn disabled(&mut self) {
        session::CompetitionSession(self).disabled().await;
    }

    async fn driver(&mut self) {
        session::CompetitionSession(self).driver().await;
    }

    async fn autonomous(&mut self) {
        session::CompetitionSession(self).autonomous().await;
    }
}

pub(super) fn record<R: OztonCompete>(
    robot: &mut R,
    sampler: &mut Sampler<()>,
    mode: CompetitionMode,
    tick: Tick,
) {
    if let Some(frame) = sampler.frame(Sample {
        mode,
        dt: tick.elapsed,
        motion: None,
    }) {
        robot.drivetrain().report(frame);
        robot.report_mechanisms(frame);
    }
}

#[cfg(all(test, not(target_os = "vexos")))]
mod tests;
