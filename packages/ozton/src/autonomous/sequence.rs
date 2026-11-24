//! Generic goal, settled-condition and timed-action lifecycle for autonomous.
use std::time::Duration;

use super::{Command, CommandError, MotionPlatform, Runner};

/// Application hooks for a generic bounded autonomous mechanism action.
/// Goal validation, subsystem selection and physical output behavior belong to the adapter.
pub trait MechanismRobot: MotionPlatform {
    /// Application-owned goal type; Ozton imposes no subsystem layout or units.
    type Target;
    fn set_target(&mut self, target: Self::Target) -> Result<(), Self::Error>;
    /// Measured position AND speed within tolerances, with valid references.
    fn at_target(&self) -> bool;
    fn set_action(&mut self, enabled: bool) -> Result<(), Self::Error>;
}

impl<R: MechanismRobot> Runner<'_, R> {
    /// Apply an application goal, wait for its settled predicate, then run a timed action.
    /// No chassis motion or field coordinates are assumed. All phases use the same
    /// cancellation guard. `timeout` bounds each phase.
    pub async fn perform(
        &mut self,
        target: R::Target,
        action_for: Duration,
        timeout: Duration,
    ) -> Result<(), CommandError<R::Error>> {
        self.perform_while(target, action_for, timeout, || {
            vexide::competition::mode() == vexide::competition::CompetitionMode::Autonomous
        })
        .await
    }

    pub async fn perform_while(
        &mut self,
        target: R::Target,
        action_for: Duration,
        timeout: Duration,
        mut enabled: impl FnMut() -> bool,
    ) -> Result<(), CommandError<R::Error>> {
        self.check_active()?;
        if action_for.is_zero() || action_for >= timeout {
            let _ = self.robot().stop();
            return Err(CommandError::Configuration);
        }
        if !enabled() {
            let _ = self.robot().stop();
            return Err(CommandError::Cancelled);
        }
        self.robot().show_status("Autonomous: prepare mechanism");
        let setup = self
            .robot()
            .set_action(false)
            .and_then(|()| self.robot().set_target(target));
        if let Err(error) = setup {
            let _ = self.robot().stop();
            return Err(CommandError::Robot(error));
        }
        let stable = self.options().settle;
        self.wait_until_while(|robot: &R| robot.at_target(), stable, timeout, &mut enabled)
        .await?;
        if let Err(error) = self.robot().set_action(true) {
            let _ = self.robot().stop();
            return Err(CommandError::Robot(error));
        }
        let result = self
            .run_while(Command::Wait(action_for), timeout, &mut enabled)
            .await;
        let stopped = self.robot().set_action(false).map_err(CommandError::Robot);
        result.and(stopped)
    }
}
