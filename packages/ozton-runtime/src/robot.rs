//! Typed composition for robots with application-owned hardware.

use vexide::{competition::CompetitionMode, prelude::Controller};

use crate::{
    competition::{Autonomous, Mechanism, RouteSet, Ui},
    diagnostics::Sampler,
};

/// Names the parts of a competition robot once, on its control policy.
/// All components are stored inline and the compiler checks the assembled layout.
pub trait RobotLayout: Sized {
    type Drivetrain;
    type Tracking;
    type Mechanisms;
    type Routes: RouteSet;
    type Report;
}

/// Concrete robot for a layout, usable in route and policy signatures.
pub type CompetitionBot<L> = Robot<
    <L as RobotLayout>::Drivetrain,
    Controller,
    <L as RobotLayout>::Tracking,
    <L as RobotLayout>::Mechanisms,
    L,
    Autonomous<<L as RobotLayout>::Routes, Sampler<<L as RobotLayout>::Report>>,
>;

/// An assembled robot. Components are stored inline and dispatched statically.
#[must_use]
pub struct Robot<D = (), C = (), T = (), X = (), P = (), A = ()> {
    pub drivetrain: D,
    pub controller: C,
    pub tracking: T,
    pub mechanisms: X,
    pub program: P,
    pub autonomous: A,
    pub(crate) driver_elapsed: std::time::Duration,
    pub(crate) driver_wrote: bool,
}

impl<D, C, T, X, P, A> Robot<D, C, T, X, P, A>
where
    D: ozton_drivetrain::model::ProgramDrive,
    P: crate::competition::Program<Self, D>,
{
    /// Elapsed time for the current driver callback.
    pub fn driver_elapsed(&self) -> std::time::Duration {
        self.driver_elapsed
    }

    /// Apply a command during the current driver callback.
    pub fn drive_driver(&mut self, command: D::DriverCommand) -> Result<(), P::Error> {
        self.drivetrain
            .apply_driver(command, self.driver_elapsed)
            .map_err(|error| P::drive_error(error, false))?;
        self.driver_wrote = true;
        Ok(())
    }
}

impl<D, C, T, X, P, A> Robot<D, C, T, X, P, A>
where
    D: ozton_drivetrain::model::ProgramDrive,
    P: crate::competition::Program<Self, D>,
    X: Mechanism<P::Error>,
{
    /// Service installed mechanisms and any additional program hooks.
    pub fn update_mechanisms(
        &mut self,
        mode: CompetitionMode,
        dt: std::time::Duration,
    ) -> Result<(), P::Error> {
        self.mechanisms.update(mode, dt)?;
        P::update_mechanisms(self, mode, dt)
    }

    /// Stop installed mechanisms and any additional program outputs.
    pub fn stop_mechanisms(&mut self) -> Result<(), P::Error> {
        self.mechanisms.stop().and(P::stop_mechanisms(self))
    }
}

/// Compose a robot before handing it to competition control.
#[must_use]
pub struct RobotBuilder<D = (), C = (), T = (), X = (), P = (), A = ()> {
    drivetrain: D,
    controller: C,
    tracking: T,
    mechanisms: X,
    program: P,
    autonomous: A,
}

impl RobotBuilder {
    pub const fn new() -> Self {
        Self {
            drivetrain: (),
            controller: (),
            tracking: (),
            mechanisms: (),
            program: (),
            autonomous: (),
        }
    }
}

impl Default for RobotBuilder {
    fn default() -> Self {
        Self::new()
    }
}

// Destructure and rebuild rather than imposing a trait or storage policy on a slot.
macro_rules! with_component {
    ($description:literal, $method:ident, $field:ident, $result:ty, $($keep:ident),+) => {
        #[doc = $description]
        pub fn $method<N>(self, $field: N) -> $result {
            let Self { $($keep,)+ .. } = self;
            RobotBuilder { $field, $($keep,)+ }
        }
    };
}

impl<D, C, T, X, P, A> RobotBuilder<D, C, T, X, P, A> {
    /// Finish assembly and hand ownership of the components to the robot.
    ///
    /// A built robot exposes only its operational interfaces.
    ///
    /// ```compile_fail
    /// use ozton_runtime::RobotBuilder;
    /// let robot = RobotBuilder::new().build();
    /// robot.with_drivetrain(());
    /// ```
    pub fn build(self) -> Robot<D, C, T, X, P, A> {
        let Self {
            drivetrain,
            controller,
            tracking,
            mechanisms,
            program,
            autonomous,
        } = self;
        Robot {
            drivetrain,
            controller,
            tracking,
            mechanisms,
            program,
            autonomous,
            driver_elapsed: std::time::Duration::ZERO,
            driver_wrote: false,
        }
    }

    with_component!("Install the owned drivetrain value.", with_drivetrain, drivetrain, RobotBuilder<N, C, T, X, P, A>, controller, tracking, mechanisms, program, autonomous);
    with_component!("Install the owned controller value.", with_controller, controller, RobotBuilder<D, N, T, X, P, A>, drivetrain, tracking, mechanisms, program, autonomous);
    with_component!("Install the owned tracking value.", with_tracking, tracking, RobotBuilder<D, C, N, X, P, A>, drivetrain, controller, mechanisms, program, autonomous);
    with_component!("Install application-wide control state and behavior.", with_program, program, RobotBuilder<D, C, T, X, N, A>, drivetrain, controller, tracking, mechanisms, autonomous);

    /// Assemble tracking from the installed drivetrain's calibration, then
    /// keep both components owned independently by the robot.
    pub async fn with_tracking_from_drive<N, E>(
        self,
        assemble: impl AsyncFnOnce(&D) -> Result<N, E>,
    ) -> Result<RobotBuilder<D, C, N, X, P, A>, E> {
        let tracking = assemble(&self.drivetrain).await?;
        Ok(self.with_tracking(tracking))
    }

    /// Register the selectable routes in menu order.
    pub fn with_auton<R: RouteSet>(
        self,
        routes: impl IntoIterator<Item = R>,
    ) -> RobotBuilder<D, C, T, X, P, AutonSelection<R>> {
        let Self {
            drivetrain,
            controller,
            tracking,
            mechanisms,
            program,
            ..
        } = self;
        RobotBuilder {
            drivetrain,
            controller,
            tracking,
            mechanisms,
            program,
            autonomous: AutonSelection(routes.into_iter().collect()),
        }
    }

    with_component!("Install the application's named mechanism aggregate.", with_mechanisms, mechanisms, RobotBuilder<D, C, T, N, P, A>, drivetrain, controller, tracking, program, autonomous);
}

/// The selectable routes before a competition UI is installed.
pub struct AutonSelection<R>(Vec<R>);

impl<D, C, T, X, P, R: RouteSet> RobotBuilder<D, C, T, X, P, AutonSelection<R>> {
    /// Attach the UI that selects the registered autonomous routes.
    pub fn with_ui(self, ui: Ui) -> RobotBuilder<D, C, T, X, P, Autonomous<R>> {
        let Self {
            drivetrain,
            controller,
            tracking,
            mechanisms,
            program,
            autonomous,
        } = self;
        RobotBuilder {
            drivetrain,
            controller,
            tracking,
            mechanisms,
            program,
            autonomous: crate::competition::autonomous_ui(ui, autonomous.0),
        }
    }
}

impl<D, C, T, X, P, R: crate::competition::RouteSet, S>
    RobotBuilder<D, C, T, X, P, crate::competition::Autonomous<R, S>>
{
    /// Attach a typed diagnostic report. USB samples use the given interval in
    /// debug builds. The SD feature also samples in release firmware.
    pub fn with_diagnostics<L>(
        self,
        interval: std::time::Duration,
    ) -> RobotBuilder<
        D,
        C,
        T,
        X,
        P,
        crate::competition::Autonomous<R, crate::diagnostics::Sampler<L>>,
    > {
        let Self {
            drivetrain,
            controller,
            tracking,
            mechanisms,
            program,
            autonomous,
        } = self;
        RobotBuilder {
            drivetrain,
            controller,
            tracking,
            mechanisms,
            program,
            autonomous: autonomous.with_diagnostics(interval),
        }
    }
}

impl<D, C, T, X, P, A> Robot<D, C, T, X, P, A>
where
    A: crate::diagnostics::Telemetry<Self>,
{
    /// Include the sample already read by the driver callback in stock diagnostics.
    pub fn record_controller(&mut self, state: &vexide::controller::ControllerState) {
        A::controller(self, Some(state));
    }

    pub fn record_diagnostics(&mut self, sample: crate::diagnostics::Sample) {
        A::record(self, sample);
    }

    /// Time from the driver tick start to the end of the wheel write.
    pub fn mark_driver_output(&mut self, tick: crate::competition::Tick) {
        #[cfg(feature = "logging")]
        A::driver_output(self, tick.work());
        #[cfg(not(feature = "logging"))]
        let _ = tick;
    }
}

impl<D, C, T, X, P, A> Robot<D, C, T, X, P, A>
where
    Self: crate::competition::CompetitionRobot + 'static,
{
    /// Run driver and autonomous competition modes for the assembled robot.
    pub async fn compete(self) -> ! {
        crate::competition::compete(self).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builder_keeps_named_hardware_inline() {
        struct Mechanisms {
            intake: i32,
            lift: i32,
        }
        let robot = RobotBuilder::new()
            .with_drivetrain(2)
            .with_mechanisms(Mechanisms { intake: 3, lift: 4 })
            .with_controller("controller")
            .build();
        assert_eq!((robot.mechanisms.intake, robot.mechanisms.lift), (3, 4));
        assert_eq!((robot.drivetrain, robot.controller), (2, "controller"));
    }
}
