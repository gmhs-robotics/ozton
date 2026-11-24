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
    type Mechanisms;
    type Routes: RouteSet;
    type Report;
    type Assists;
}

/// Concrete robot for a layout, usable in route and policy signatures.
pub type CompetitionBot<L> = Robot<
    <L as RobotLayout>::Drivetrain,
    Controller,
    <L as RobotLayout>::Mechanisms,
    L,
    Autonomous<<L as RobotLayout>::Routes, Sampler<<L as RobotLayout>::Report>>,
    <L as RobotLayout>::Assists,
>;

/// An assembled robot. Components are stored inline and dispatched statically.
#[must_use]
pub struct Robot<D = (), C = (), X = (), P = (), A = (), S = ()> {
    pub drivetrain: D,
    pub controller: C,
    pub mechanisms: X,
    pub program: P,
    pub autonomous: A,
    pub assists: S,
    pub(crate) driver_elapsed: std::time::Duration,
    pub(crate) driver_wrote: bool,
}

impl<D, C, X, P, A, S> Robot<D, C, X, P, A, S>
where
    D: crate::competition::ProgramDrive,
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

impl<D, C, X, P, A, S> Robot<D, C, X, P, A, S>
where
    D: crate::competition::ProgramDrive,
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
pub struct RobotBuilder<D = (), C = (), X = (), P = (), A = (), S = ()> {
    drivetrain: D,
    controller: C,
    mechanisms: X,
    program: P,
    autonomous: A,
    assists: S,
}

impl RobotBuilder {
    pub const fn new() -> Self {
        Self {
            drivetrain: (),
            controller: (),
            mechanisms: (),
            program: (),
            autonomous: (),
            assists: (),
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

impl<D, C, X, P, A, S> RobotBuilder<D, C, X, P, A, S> {
    /// Finish assembly and hand ownership of the components to the robot.
    ///
    /// A built robot exposes only its operational interfaces.
    ///
    /// ```compile_fail
    /// use ozton_runtime::RobotBuilder;
    /// let robot = RobotBuilder::new().build();
    /// robot.with_drivetrain(());
    /// ```
    pub fn build(self) -> Robot<D, C, X, P, A, S> {
        let Self {
            drivetrain,
            controller,
            mechanisms,
            program,
            autonomous,
            assists,
        } = self;
        Robot {
            drivetrain,
            controller,
            mechanisms,
            program,
            autonomous,
            assists,
            driver_elapsed: std::time::Duration::ZERO,
            driver_wrote: false,
        }
    }

    with_component!("Install the owned drivetrain value.", with_drivetrain, drivetrain, RobotBuilder<N, C, X, P, A, S>, controller, mechanisms, program, autonomous, assists);
    with_component!("Install the owned controller value.", with_controller, controller, RobotBuilder<D, N, X, P, A, S>, drivetrain, mechanisms, program, autonomous, assists);
    with_component!("Install application-wide control state and behavior.", with_program, program, RobotBuilder<D, C, X, N, A, S>, drivetrain, controller, mechanisms, autonomous, assists);

    with_component!("Install the owned drive assist stack.", with_drive_assists, assists, RobotBuilder<D, C, X, P, A, N>, drivetrain, controller, mechanisms, program, autonomous);

    /// Register the selectable routes in menu order.
    pub fn with_auton<R: RouteSet>(
        self,
        routes: impl IntoIterator<Item = R>,
    ) -> RobotBuilder<D, C, X, P, AutonSelection<R>, S> {
        let Self {
            drivetrain,
            controller,
            mechanisms,
            program,
            assists,
            ..
        } = self;
        RobotBuilder {
            drivetrain,
            controller,
            mechanisms,
            program,
            assists,
            autonomous: AutonSelection(routes.into_iter().collect()),
        }
    }

    with_component!("Install the application's named mechanism aggregate.", with_mechanisms, mechanisms, RobotBuilder<D, C, N, P, A, S>, drivetrain, controller, program, autonomous, assists);
}

/// The selectable routes before a competition UI is installed.
pub struct AutonSelection<R>(Vec<R>);

impl<D, C, X, P, R: RouteSet, S> RobotBuilder<D, C, X, P, AutonSelection<R>, S> {
    /// Attach the UI that selects the registered autonomous routes.
    pub fn with_ui(self, ui: Ui<Option<R>>) -> RobotBuilder<D, C, X, P, Autonomous<R>, S> {
        let Self {
            drivetrain,
            controller,
            mechanisms,
            program,
            autonomous,
            assists,
        } = self;
        RobotBuilder {
            drivetrain,
            controller,
            mechanisms,
            program,
            assists,
            autonomous: crate::competition::autonomous_ui(ui, autonomous.0),
        }
    }
}

impl<D, C, X, P, R: crate::competition::RouteSet, S, B>
    RobotBuilder<D, C, X, P, crate::competition::Autonomous<R, S>, B>
{
    /// Register the driver callback alongside the autonomous routes.
    pub fn with_driver(mut self, driver: R) -> Self
    where
        P: crate::competition::CompetitionLayout<Routes = R>,
    {
        self.autonomous.set_driver(driver);
        self
    }

    /// Attach a typed diagnostic report. USB samples use the given interval in
    /// debug builds. The SD feature also samples in release firmware.
    pub fn with_diagnostics<L>(
        self,
        interval: std::time::Duration,
    ) -> RobotBuilder<
        D,
        C,
        X,
        P,
        crate::competition::Autonomous<R, crate::diagnostics::Sampler<L>>,
        B,
    > {
        let Self {
            drivetrain,
            controller,
            mechanisms,
            program,
            autonomous,
            assists,
        } = self;
        RobotBuilder {
            drivetrain,
            controller,
            mechanisms,
            program,
            assists,
            autonomous: autonomous.with_diagnostics(interval),
        }
    }
}

impl<D, C, X, P, A, S> Robot<D, C, X, P, A, S>
where
    A: crate::diagnostics::Telemetry<Self>,
{
    pub fn record_diagnostics(&mut self, sample: crate::diagnostics::Sample) {
        #[cfg(feature = "logging")]
        A::record(self, sample);
        #[cfg(not(feature = "logging"))]
        let _ = sample;
    }

    /// Time from the driver tick start to the end of the wheel write.
    pub fn mark_driver_output(&mut self, tick: crate::competition::Tick) {
        #[cfg(feature = "logging")]
        if ozton_diagnostics::serial::sd_enabled() {
            A::driver_output(self, tick.work());
        }
        #[cfg(not(feature = "logging"))]
        let _ = tick;
    }
}

impl<D, C, X, P, A, S> Robot<D, C, X, P, A, S>
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
