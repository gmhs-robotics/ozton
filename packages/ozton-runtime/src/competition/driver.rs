//! Pacing and output supervision for an application-owned driver loop.

use std::time::Instant;

use super::{Autonomous, CompetitionMode, OztonCompete, ProgramDrive, RobotRoute, Tick, runtime};
use crate::{
    clock::sleep,
    competition::{Mechanism, session::control_sleep},
    diagnostics::Sampler,
};

/// The competition bookkeeping for one invocation of the application's driver.
pub struct DriverLoop<'a, R: OztonCompete> {
    autonomous: &'a mut Autonomous<RobotRoute<R, R::Error>, Sampler<()>>,
    tick: Tick,
}

impl<'a, R: OztonCompete> DriverLoop<'a, R> {
    pub(super) fn new(
        autonomous: &'a mut Autonomous<RobotRoute<R, R::Error>, Sampler<()>>,
    ) -> Self {
        Self {
            autonomous,
            tick: Tick::start(Default::default()),
        }
    }

    /// Call once after each iteration's commands, with hardware borrows released.
    /// A failure should be returned from the driver so competition can stop outputs.
    pub async fn finish_iteration(&mut self, robot: &mut R) -> Result<(), R::Error> {
        if robot.drivetrain().take_driver_output() {
            self.autonomous.sampler.driver_output(self.tick.work());
        } else {
            robot.drivetrain().stop()?;
        }
        robot
            .mechanisms()
            .update(CompetitionMode::Driver, self.tick.elapsed)?;
        self.autonomous.menu.poll(false);
        runtime::record(
            robot,
            &mut self.autonomous.sampler,
            CompetitionMode::Driver,
            self.tick,
        );
        sleep(control_sleep(R::CONTROL_PERIOD, self.tick.work())).await;
        self.tick = Tick::start(Instant::now().saturating_duration_since(self.tick.started));
        Ok(())
    }
}
