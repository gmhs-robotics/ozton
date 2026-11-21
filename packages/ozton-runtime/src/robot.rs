//! Typed composition for robots with stock or application-owned hardware.

use vexide::prelude::Controller;

use crate::{
    competition::{Autonomous, RouteSet, Ui},
    diagnostics::Sampler,
};

/// Names the parts of a competition robot once, on its control policy.
/// All components are stored inline and the compiler checks the assembled layout.
pub trait RobotLayout: Sized {
    type Drivetrain;
    type Tracking;
    type Subsystems;
    type Routes: RouteSet;
    type Report;
}

/// Concrete robot for a layout, usable in route and policy signatures.
pub type CompetitionBot<L> = Robot<
    <L as RobotLayout>::Drivetrain,
    Controller,
    <L as RobotLayout>::Tracking,
    <L as RobotLayout>::Subsystems,
    L,
    Autonomous<<L as RobotLayout>::Routes, Sampler<<L as RobotLayout>::Report>>,
>;

/// An application-owned subsystem. The tick input can hold controller state
/// and commands from the robot's control policy.
pub trait Subsystem<Input, Error> {
    fn update(&mut self, input: &mut Input) -> Result<(), Error>;

    /// Bring this subsystem's outputs to a safe state.
    fn stop(&mut self) -> Result<(), Error> {
        Ok(())
    }
}

/// Lifecycle of the subsystem list built by [`RobotBuilder::add_subsystem`].
pub trait SubsystemList<Input, Error> {
    fn update_all(&mut self, input: &mut Input) -> Result<(), Error>;
    fn stop_all(&mut self) -> Result<(), Error>;
}

impl<I, E> SubsystemList<I, E> for () {
    fn update_all(&mut self, _: &mut I) -> Result<(), E> {
        Ok(())
    }

    fn stop_all(&mut self) -> Result<(), E> {
        Ok(())
    }
}

impl<I, E, A: SubsystemList<I, E>, B: Subsystem<I, E>> SubsystemList<I, E> for (A, B) {
    fn update_all(&mut self, input: &mut I) -> Result<(), E> {
        self.0.update_all(input)?;
        self.1.update(input)
    }

    fn stop_all(&mut self) -> Result<(), E> {
        self.0.stop_all().and(self.1.stop())
    }
}

/// An assembled robot. Components are stored inline and dispatched statically.
#[must_use]
pub struct Robot<D = (), C = (), T = (), X = (), P = (), A = ()> {
    pub drivetrain: D,
    pub controller: C,
    pub tracking: T,
    pub subsystems: X,
    pub controls: P,
    pub autonomous: A,
}

impl<D, C, T, S, P, A> Robot<D, C, T, ((), S), P, A> {
    /// Access the single installed subsystem.
    pub fn subsystem(&mut self) -> &mut S {
        &mut self.subsystems.1
    }
}

/// Compose a robot before handing it to competition control.
#[must_use]
pub struct RobotBuilder<D = (), C = (), T = (), X = (), P = (), A = ()> {
    drivetrain: D,
    controller: C,
    tracking: T,
    subsystems: X,
    controls: P,
    autonomous: A,
}

impl RobotBuilder {
    pub const fn new() -> Self {
        Self {
            drivetrain: (),
            controller: (),
            tracking: (),
            subsystems: (),
            controls: (),
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
            subsystems,
            controls,
            autonomous,
        } = self;
        Robot {
            drivetrain,
            controller,
            tracking,
            subsystems,
            controls,
            autonomous,
        }
    }

    with_component!("Install the owned drivetrain value.", with_drivetrain, drivetrain, RobotBuilder<N, C, T, X, P, A>, controller, tracking, subsystems, controls, autonomous);
    with_component!("Install the owned controller value.", with_controller, controller, RobotBuilder<D, N, T, X, P, A>, drivetrain, tracking, subsystems, controls, autonomous);
    with_component!("Install the owned tracking value.", with_tracking, tracking, RobotBuilder<D, C, N, X, P, A>, drivetrain, controller, subsystems, controls, autonomous);
    with_component!("Install application-wide control state and behavior.", with_controls, controls, RobotBuilder<D, C, T, X, N, A>, drivetrain, controller, tracking, subsystems, autonomous);

    /// Register the selectable routes in menu order.
    pub fn with_auton<R: RouteSet>(
        self,
        routes: impl IntoIterator<Item = R>,
    ) -> RobotBuilder<D, C, T, X, P, AutonSelection<R>> {
        let Self {
            drivetrain,
            controller,
            tracking,
            subsystems,
            controls,
            ..
        } = self;
        RobotBuilder {
            drivetrain,
            controller,
            tracking,
            subsystems,
            controls,
            autonomous: AutonSelection(routes.into_iter().collect()),
        }
    }

    /// Append a subsystem to the statically dispatched update list.
    pub fn add_subsystem<N>(self, subsystem: N) -> RobotBuilder<D, C, T, (X, N), P, A> {
        let Self {
            drivetrain,
            controller,
            tracking,
            subsystems,
            controls,
            autonomous,
        } = self;
        RobotBuilder {
            drivetrain,
            controller,
            tracking,
            subsystems: (subsystems, subsystem),
            controls,
            autonomous,
        }
    }
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
            subsystems,
            controls,
            autonomous,
        } = self;
        RobotBuilder {
            drivetrain,
            controller,
            tracking,
            subsystems,
            controls,
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
            subsystems,
            controls,
            autonomous,
        } = self;
        RobotBuilder {
            drivetrain,
            controller,
            tracking,
            subsystems,
            controls,
            autonomous: autonomous.with_diagnostics(interval),
        }
    }
}

impl<D, C, T, X, P, A> Robot<D, C, T, X, P, A>
where
    A: crate::diagnostics::Telemetry<Self>,
{
    pub fn record_diagnostics(&mut self, sample: crate::diagnostics::Sample) {
        A::record(self, sample);
    }

    /// Time from the driver tick start to the end of the wheel write.
    pub fn mark_driver_output(&mut self, tick: crate::competition::Tick) {
        #[cfg(any(debug_assertions, feature = "sd-logging"))]
        A::driver_output(self, tick.work());
        #[cfg(not(any(debug_assertions, feature = "sd-logging")))]
        let _ = tick;
    }
}

impl<D, C, T, X, P, A> Robot<D, C, T, X, P, A>
where
    Self: crate::competition::CompetitionRobot,
{
    /// Run driver and autonomous competition modes for the assembled robot.
    pub async fn compete(self) -> ! {
        crate::competition::compete(self).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct First;
    impl Subsystem<i32, &'static str> for First {
        fn update(&mut self, input: &mut i32) -> Result<(), &'static str> {
            if *input < 0 {
                Err("update failed")
            } else {
                Ok(())
            }
        }

        fn stop(&mut self) -> Result<(), &'static str> {
            Err("first stop failed")
        }
    }

    #[derive(Default)]
    struct Read {
        input: i32,
        stopped: bool,
    }
    impl Subsystem<i32, &'static str> for Read {
        fn update(&mut self, input: &mut i32) -> Result<(), &'static str> {
            self.input = *input;
            Ok(())
        }

        fn stop(&mut self) -> Result<(), &'static str> {
            self.stopped = true;
            Ok(())
        }
    }

    struct Independent;
    impl Subsystem<i32, &'static str> for Independent {
        fn update(&mut self, _: &mut i32) -> Result<(), &'static str> {
            Ok(())
        }
    }

    #[test]
    fn customs_update_in_order_and_all_stop_after_failure() {
        let mut robot = RobotBuilder::new()
            .with_drivetrain(2)
            .add_subsystem(First)
            .add_subsystem(Independent)
            .with_controller("controller")
            .add_subsystem(Read::default())
            .build();

        assert_eq!(robot.subsystems.update_all(&mut 3), Ok(()));
        assert_eq!(robot.subsystems.1.input, 3);
        assert_eq!(robot.subsystems.update_all(&mut -1), Err("update failed"));
        assert_eq!(robot.subsystems.1.input, 3);
        assert_eq!((robot.drivetrain, robot.controller), (2, "controller"));
        assert_eq!(robot.subsystems.stop_all(), Err("first stop failed"));
        assert!(robot.subsystems.1.stopped);
        assert_eq!(
            std::mem::size_of::<Robot<(), (), (), ((), Independent)>>(),
            0
        );
    }
}
