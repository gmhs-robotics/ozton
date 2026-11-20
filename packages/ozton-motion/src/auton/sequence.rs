//! Generic goal, settled-condition and timed-action lifecycle for autonomous.
use std::time::Duration;

use super::{AutonError, AutonRobot, Command, Runner};

/// Application hooks for a generic bounded autonomous mechanism action.
/// Goal validation, subsystem selection and physical output behavior belong to the adapter.
pub trait MechanismRobot: AutonRobot {
    /// Application-owned goal type; Ozton imposes no subsystem layout or units.
    type Target;
    fn set_target(&mut self, target: Self::Target) -> Result<(), Self::Error>;
    /// Measured position AND speed within tolerances, with valid references.
    fn at_target(&self) -> bool;
    fn set_action(&mut self, enabled: bool) -> Result<(), Self::Error>;
}

impl<R: MechanismRobot> Runner<'_, R> {
    /// Apply an application goal, wait for its settled predicate, then run a timed action.
    /// No chassis motion or field coordinates are assumed. All phases share the
    /// route deadline and cancellation guards. `timeout` bounds each phase.
    pub async fn perform(
        &mut self,
        target: R::Target,
        action_for: Duration,
        timeout: Duration,
    ) -> Result<(), AutonError<R::Error>> {
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
    ) -> Result<(), AutonError<R::Error>> {
        self.check_active()?;
        if action_for.is_zero() || action_for >= timeout {
            let _ = self.robot().stop();
            return Err(AutonError::Configuration);
        }
        if !enabled() {
            let _ = self.robot().stop();
            return Err(AutonError::Cancelled);
        }
        self.robot().show_status("Autonomous: prepare mechanism");
        let setup = self
            .robot()
            .set_action(false)
            .and_then(|()| self.robot().set_target(target));
        if let Err(error) = setup {
            let _ = self.robot().stop();
            return Err(AutonError::Robot(error));
        }
        self.run_while(
            Command::WaitUntil(|robot: &R| robot.at_target()),
            timeout,
            &mut enabled,
        )
        .await?;
        if let Err(error) = self.robot().set_action(true) {
            let _ = self.robot().stop();
            return Err(AutonError::Robot(error));
        }
        let result = self
            .run_while(Command::Wait(action_for), timeout, &mut enabled)
            .await;
        let stopped = self.robot().set_action(false).map_err(AutonError::Robot);
        result.and(stopped)
    }
}

/// Goal/action adapter for the stock Ozton robot. Implement on the last registered subsystem;
/// the chassis, selector, cancellation and route clock stay in Ozton.
pub trait MechanismActions {
    type Target;
    fn set_target(&mut self, target: Self::Target) -> Result<(), vexide::smart::PortError>;
    fn at_target(&self) -> bool;
    fn set_action(&mut self, enabled: bool) -> Result<(), vexide::smart::PortError>;
}
impl<A, M: MechanismActions> MechanismRobot for super::Robot<(A, M)>
where
    (A, M): for<'a> ozton_runtime::CustomList<super::StockTick<'a>, vexide::smart::PortError>,
{
    type Target = M::Target;
    fn set_target(&mut self, target: Self::Target) -> Result<(), Self::Error> {
        self.custom
            .1
            .set_target(target)
            .map_err(super::chassis::ChassisError::Output)
    }
    fn at_target(&self) -> bool {
        self.custom.1.at_target()
    }
    fn set_action(&mut self, enabled: bool) -> Result<(), Self::Error> {
        self.custom
            .1
            .set_action(enabled)
            .map_err(super::chassis::ChassisError::Output)
    }
}
