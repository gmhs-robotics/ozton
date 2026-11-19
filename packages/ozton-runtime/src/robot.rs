//! Typed composition for robots with stock or application-owned hardware.

/// A configured driver control that can change the concrete drivetrain before
/// the application sends outputs. The input may be a controller state or any
/// application-defined command.
pub trait DriveControl<Drivetrain, Input> {
    fn update(&mut self, input: &Input, drivetrain: &mut Drivetrain);
}

impl<D, I> DriveControl<D, I> for () {
    fn update(&mut self, _: &I, _: &mut D) {}
}

impl<D, I, A: DriveControl<D, I>, B: DriveControl<D, I>> DriveControl<D, I> for (A, B) {
    fn update(&mut self, input: &I, drivetrain: &mut D) {
        self.0.update(input, drivetrain);
        self.1.update(input, drivetrain);
    }
}

/// Assemble owned hardware and custom state as ordinary fields. Each builder
/// method changes only the type of the field it replaces. An omitted component
/// is unit, and the assembly adds no allocation or dynamic dispatch.
#[must_use]
pub struct Robot<D = (), C = (), T = (), M = (), X = (), P = (), A = ()> {
    pub drivetrain: D,
    pub controller: C,
    pub tracking: T,
    pub mechanisms: M,
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
            mechanisms: (),
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

impl<D, C, T, M, X, P, A> Robot<D, C, T, M, X, P, A> {
    with_component!("Install the owned drivetrain value.", with_drivetrain, drivetrain, Robot<N, C, T, M, X, P, A>, controller, tracking, mechanisms, custom, controls, autonomous);
    with_component!("Install the owned controller value.", with_controller, controller, Robot<D, N, T, M, X, P, A>, drivetrain, tracking, mechanisms, custom, controls, autonomous);
    with_component!("Install the owned tracking value.", with_tracking, tracking, Robot<D, C, N, M, X, P, A>, drivetrain, controller, mechanisms, custom, controls, autonomous);
    with_component!("Install the owned mechanisms value.", with_mechanisms, mechanisms, Robot<D, C, T, N, X, P, A>, drivetrain, controller, tracking, custom, controls, autonomous);
    with_component!("Install a driver policy or custom state.", with_custom, custom, Robot<D, C, T, M, N, P, A>, drivetrain, controller, tracking, mechanisms, controls, autonomous);
    with_component!("Install application-wide control state and behavior.", with_controls, controls, Robot<D, C, T, M, X, N, A>, drivetrain, controller, tracking, mechanisms, custom, autonomous);

    /// Install the Brain menu and a statically dispatched route set.
    pub fn with_autonomous<R: crate::competition::RouteSet>(
        self,
        display: vexide::display::Display,
    ) -> Robot<D, C, T, M, X, P, crate::competition::Autonomous<R>> {
        let Self {
            drivetrain,
            controller,
            tracking,
            mechanisms,
            custom,
            controls,
            ..
        } = self;
        Robot {
            drivetrain,
            controller,
            tracking,
            mechanisms,
            custom,
            controls,
            autonomous: crate::competition::autonomous_menu(display),
        }
    }

    /// Append a driver control. When applied, controls run in registration order.
    pub fn with_control<N>(self, control: N) -> Robot<D, C, T, M, (X, N), P, A> {
        let Self {
            drivetrain,
            controller,
            tracking,
            mechanisms,
            custom,
            controls,
            autonomous,
        } = self;
        Robot {
            drivetrain,
            controller,
            tracking,
            mechanisms,
            custom: (custom, control),
            controls,
            autonomous,
        }
    }

    /// Run the configured policy against the actual drivetrain. Applications may
    /// also access all fields directly and supply their own control loop.
    pub fn apply_custom<I>(&mut self, input: &I)
    where
        X: DriveControl<D, I>,
    {
        self.custom.update(input, &mut self.drivetrain);
    }
}

impl<D, C, T, M, X, P, R: crate::competition::RouteSet, S>
    Robot<D, C, T, M, X, P, crate::competition::Autonomous<R, S>>
{
    /// Attach a typed diagnostic report to the Brain runtime. Periodic serial
    /// output is active in debug builds and samples after control outputs.
    pub fn with_diagnostics<L>(
        self,
        interval: std::time::Duration,
    ) -> Robot<D, C, T, M, X, P, crate::competition::Autonomous<R, crate::diagnostics::Sampler<L>>>
    {
        let Self {
            drivetrain,
            controller,
            tracking,
            mechanisms,
            custom,
            controls,
            autonomous,
        } = self;
        Robot {
            drivetrain,
            controller,
            tracking,
            mechanisms,
            custom,
            controls,
            autonomous: autonomous.with_diagnostics(interval),
        }
    }
}

impl<D, C, T, M, X, P, A> Robot<D, C, T, M, X, P, A>
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

impl<D, C, T, M, X, P, A> Robot<D, C, T, M, X, P, A>
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

    struct Scale;
    impl DriveControl<i32, i32> for Scale {
        fn update(&mut self, input: &i32, drive: &mut i32) {
            *drive *= input;
        }
    }

    struct Add;
    impl DriveControl<i32, i32> for Add {
        fn update(&mut self, input: &i32, drive: &mut i32) {
            *drive += input;
        }
    }

    #[test]
    fn builders_keep_other_parts_and_custom_control_updates_in_place() {
        let mut robot = Robot::new()
            .with_drivetrain(2)
            .with_controller("controller")
            .with_tracking(true)
            .with_mechanisms([0; 2])
            .with_custom(Scale)
            .with_controls("application")
            .with_control(Add);
        robot.apply_custom(&3);
        assert_eq!(robot.drivetrain, 9);
        assert_eq!(robot.controls, "application");
        assert_eq!(
            (robot.controller, robot.tracking, robot.mechanisms),
            ("controller", true, [0; 2])
        );
        assert_eq!(std::mem::size_of::<Robot<(), (), (), (), Scale>>(), 0);
    }
}
