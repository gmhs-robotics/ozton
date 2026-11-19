//! Typed composition for robots with stock or application-owned hardware.

/// An application-owned subsystem. The tick input can hold controller state
/// and shared commands. Earlier customs can be borrowed directly through
/// [`CustomAt`]. Independent customs can leave `Earlier` unconstrained.
pub trait Custom<Input, Error, Earlier = ()> {
    fn update(&mut self, input: &mut Input, earlier: &mut Earlier) -> Result<(), Error>;

    /// Bring this subsystem's outputs to a safe state.
    fn stop(&mut self) -> Result<(), Error> {
        Ok(())
    }
}

/// Select the most recently registered custom in a list.
pub struct Latest;

/// Select an index in the prefix before the most recently registered custom.
pub struct Previous<Index>(core::marker::PhantomData<Index>);

/// Borrow a registered subsystem at a compile-time index.
pub trait CustomAt<Index> {
    type Value;
    fn get(&mut self) -> &mut Self::Value;
}

impl<Earlier, Last> CustomAt<Latest> for (Earlier, Last) {
    type Value = Last;

    fn get(&mut self) -> &mut Last {
        &mut self.1
    }
}

impl<Earlier: CustomAt<Index>, Last, Index> CustomAt<Previous<Index>> for (Earlier, Last) {
    type Value = Earlier::Value;

    fn get(&mut self) -> &mut Self::Value {
        self.0.get()
    }
}

/// Lifecycle of the subsystem list built by [`Robot::with_custom`].
pub trait CustomList<Input, Error> {
    fn update_all(&mut self, input: &mut Input) -> Result<(), Error>;
    fn stop_all(&mut self) -> Result<(), Error>;
}

impl<I, E> CustomList<I, E> for () {
    fn update_all(&mut self, _: &mut I) -> Result<(), E> {
        Ok(())
    }

    fn stop_all(&mut self) -> Result<(), E> {
        Ok(())
    }
}

impl<I, E, A: CustomList<I, E>, B: Custom<I, E, A>> CustomList<I, E> for (A, B) {
    fn update_all(&mut self, input: &mut I) -> Result<(), E> {
        self.0.update_all(input)?;
        self.1.update(input, &mut self.0)
    }

    fn stop_all(&mut self) -> Result<(), E> {
        self.0.stop_all().and(self.1.stop())
    }
}

/// Assemble hardware and custom subsystems as owned fields. Omitted components
/// are unit. The custom list is stored inline and dispatched statically.
#[must_use]
pub struct Robot<D = (), C = (), T = (), X = (), P = (), A = ()> {
    pub drivetrain: D,
    pub controller: C,
    pub tracking: T,
    pub custom: X,
    pub controls: P,
    pub autonomous: A,
}

impl Robot {
    pub const fn new() -> Self {
        Self {
            drivetrain: (),
            controller: (),
            tracking: (),
            custom: (),
            controls: (),
            autonomous: (),
        }
    }
}

impl Default for Robot {
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
            Robot { $field, $($keep,)+ }
        }
    };
}

impl<D, C, T, X, P, A> Robot<D, C, T, X, P, A> {
    with_component!("Install the owned drivetrain value.", with_drivetrain, drivetrain, Robot<N, C, T, X, P, A>, controller, tracking, custom, controls, autonomous);
    with_component!("Install the owned controller value.", with_controller, controller, Robot<D, N, T, X, P, A>, drivetrain, tracking, custom, controls, autonomous);
    with_component!("Install the owned tracking value.", with_tracking, tracking, Robot<D, C, N, X, P, A>, drivetrain, controller, custom, controls, autonomous);
    with_component!("Install application-wide control state and behavior.", with_controls, controls, Robot<D, C, T, X, N, A>, drivetrain, controller, tracking, custom, autonomous);

    /// Install the Brain menu and a statically dispatched route set.
    pub fn with_autonomous<R: crate::competition::RouteSet>(
        self,
        display: vexide::display::Display,
    ) -> Robot<D, C, T, X, P, crate::competition::Autonomous<R>> {
        let Self {
            drivetrain,
            controller,
            tracking,
            custom,
            controls,
            ..
        } = self;
        Robot {
            drivetrain,
            controller,
            tracking,
            custom,
            controls,
            autonomous: crate::competition::autonomous_menu(display),
        }
    }

    /// Append a subsystem to the custom tick chain. Earlier subsystems run first.
    ///
    /// Missing dependencies are rejected when the chain is used.
    ///
    /// ```compile_fail
    /// use ozton_runtime::{Custom, CustomAt, CustomList, Latest, Robot};
    ///
    /// struct Intake;
    /// struct Feeder;
    /// impl<E: CustomAt<Latest, Value = Intake>> Custom<(), (), E> for Feeder {
    ///     fn update(&mut self, _: &mut (), earlier: &mut E) -> Result<(), ()> {
    ///         let _intake = earlier.get();
    ///         Ok(())
    ///     }
    /// }
    ///
    /// Robot::new().with_custom(Feeder).custom.update_all(&mut ()).unwrap();
    /// ```
    pub fn with_custom<N>(self, subsystem: N) -> Robot<D, C, T, (X, N), P, A> {
        let Self {
            drivetrain,
            controller,
            tracking,
            custom,
            controls,
            autonomous,
        } = self;
        Robot {
            drivetrain,
            controller,
            tracking,
            custom: (custom, subsystem),
            controls,
            autonomous,
        }
    }
}

impl<D, C, T, X, P, R: crate::competition::RouteSet, S>
    Robot<D, C, T, X, P, crate::competition::Autonomous<R, S>>
{
    /// Attach a typed diagnostic report to the Brain runtime. Periodic serial
    /// output is active in debug builds and samples after control outputs.
    pub fn with_diagnostics<L>(
        self,
        interval: std::time::Duration,
    ) -> Robot<D, C, T, X, P, crate::competition::Autonomous<R, crate::diagnostics::Sampler<L>>>
    {
        let Self {
            drivetrain,
            controller,
            tracking,
            custom,
            controls,
            autonomous,
        } = self;
        Robot {
            drivetrain,
            controller,
            tracking,
            custom,
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
        #[cfg(debug_assertions)]
        A::driver_output(self, tick.work());
        #[cfg(not(debug_assertions))]
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

    #[derive(Default)]
    struct Publish(i32);
    impl Custom<i32, &'static str> for Publish {
        fn update(&mut self, input: &mut i32, _: &mut ()) -> Result<(), &'static str> {
            self.0 = *input * 2;
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
    impl<Earlier: CustomAt<Previous<Latest>, Value = Publish>> Custom<i32, &'static str, Earlier>
        for Read
    {
        fn update(&mut self, _: &mut i32, earlier: &mut Earlier) -> Result<(), &'static str> {
            self.input = earlier.get().0;
            Ok(())
        }

        fn stop(&mut self) -> Result<(), &'static str> {
            self.stopped = true;
            Ok(())
        }
    }

    struct Independent;
    impl<Earlier> Custom<i32, &'static str, Earlier> for Independent {
        fn update(&mut self, _: &mut i32, _: &mut Earlier) -> Result<(), &'static str> {
            Ok(())
        }
    }

    #[test]
    fn customs_borrow_earlier_subsystems_and_all_stop_after_failure() {
        let mut robot = Robot::new()
            .with_drivetrain(2)
            .with_custom(Publish::default())
            .with_custom(Independent)
            .with_controller("controller")
            .with_custom(Read::default());

        assert_eq!(robot.custom.update_all(&mut 3), Ok(()));
        assert_eq!(robot.custom.1.input, 6);
        assert_eq!(robot.custom.update_all(&mut -1), Err("update failed"));
        assert_eq!(robot.custom.1.input, 6);
        assert_eq!((robot.drivetrain, robot.controller), (2, "controller"));
        assert_eq!(robot.custom.stop_all(), Err("first stop failed"));
        assert!(robot.custom.1.stopped);
        assert_eq!(
            std::mem::size_of::<Robot<(), (), (), ((), Independent)>>(),
            0
        );
    }
}
