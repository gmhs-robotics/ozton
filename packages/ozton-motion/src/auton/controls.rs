//! Motion hooks for an assembled Ozton robot.
use std::time::Duration;

use ozton_math::{
    geometry::Pose,
    uom::si::f64::{Length, Velocity},
};
use ozton_runtime::{
    Robot, SubsystemList,
    diagnostics::{MotionSample, Sample, Telemetry},
};

use super::{AutonRobot, Feedback, MechanismRobot};

/// Connect the installed control policy to the autonomous runner. Each hook
/// receives the assembled robot so feedback and outputs can use its concrete
/// devices. The runner owns timing, cancellation, retries and motion algorithms.
pub trait AutonomousControls<R> {
    type Error;
    type SubsystemInput;

    /// Physical chassis options for the high-level route commands.
    fn drive_options(_robot: &R) -> Option<super::DriveOptions> {
        None
    }

    fn session_limits(_robot: &R) -> super::routine_settings::SessionLimits {
        super::routine_settings::SessionLimits::MATCH
    }

    fn motion_period(_robot: &R) -> Duration {
        ozton_runtime::clock::CONTROL_PERIOD
    }

    fn maximum_motion_gap(_robot: &R) -> Duration {
        Duration::from_millis(100)
    }

    /// Capture one feedback sample. None requests a neutral rebaseline tick.
    fn sample(robot: &mut R) -> Result<Option<Feedback>, Self::Error>;
    /// Supply shared input for the registered subsystems on a runner tick.
    fn subsystem_input(robot: &mut R, dt: Duration) -> Result<Self::SubsystemInput, Self::Error>;
    fn drive(robot: &mut R, wheel_speeds: [Velocity; 2], dt: Duration) -> Result<(), Self::Error>;
    fn stop_drive(robot: &mut R) -> Result<(), Self::Error>;
    /// Stop application-controlled outputs and clear retained goals.
    fn stop(robot: &mut R) -> Result<(), Self::Error>;
    fn set_pose(robot: &mut R, pose: Pose) -> Result<(), Self::Error>;
    fn request_distance_reset(robot: &mut R);

    fn pose_trusted(_robot: &R) -> bool {
        false
    }
    fn recoverable(_robot: &R, _error: &Self::Error) -> bool {
        false
    }
    fn can_skip_failure(_robot: &R, _error: &Self::Error) -> bool {
        false
    }
    fn pause(robot: &mut R) -> Result<(), Self::Error> {
        Self::stop(robot)
    }
    fn cancel_distance_reset(_robot: &mut R) {}
    fn log_step(
        _robot: &mut R,
        _elapsed: Duration,
        _dt: Duration,
        _progress: Length,
        _wheels: [Velocity; 2],
    ) {
    }
}

impl<D, C, T, X, P, A> AutonRobot for Robot<D, C, T, X, P, A>
where
    P: AutonomousControls<Self>,
    A: Telemetry<Self>,
    X: SubsystemList<P::SubsystemInput, P::Error>,
{
    type Error = P::Error;

    fn sample(&mut self) -> Result<Option<Feedback>, Self::Error> {
        P::sample(self)
    }
    fn update_subsystems(&mut self, dt: Duration) -> Result<(), Self::Error> {
        let mut input = P::subsystem_input(self, dt)?;
        self.subsystems.update_all(&mut input)
    }
    fn drive(&mut self, wheel_speeds: [Velocity; 2], dt: Duration) -> Result<(), Self::Error> {
        P::drive(self, wheel_speeds, dt)
    }
    fn stop_drive(&mut self) -> Result<(), Self::Error> {
        P::stop_drive(self)
    }
    fn stop(&mut self) -> Result<(), Self::Error> {
        P::stop(self).and(self.subsystems.stop_all())
    }
    fn set_pose(&mut self, pose: Pose) -> Result<(), Self::Error> {
        P::set_pose(self, pose)
    }
    fn request_distance_reset(&mut self) {
        P::request_distance_reset(self);
    }
    fn pose_trusted(&self) -> bool {
        P::pose_trusted(self)
    }
    fn recoverable(&self, error: &Self::Error) -> bool {
        P::recoverable(self, error)
    }
    fn can_skip_failure(&self, error: &Self::Error) -> bool {
        P::can_skip_failure(self, error)
    }
    fn pause(&mut self) -> Result<(), Self::Error> {
        P::pause(self).and(self.subsystems.stop_all())
    }
    fn cancel_distance_reset(&mut self) {
        P::cancel_distance_reset(self);
    }
    fn show_status(&mut self, status: &str) {
        A::status(self, status);
    }

    fn log_step(
        &mut self,
        elapsed: Duration,
        dt: Duration,
        progress: Length,
        wheels: [Velocity; 2],
    ) {
        P::log_step(self, elapsed, dt, progress, wheels);
        self.record_diagnostics(Sample {
            mode: vexide::competition::CompetitionMode::Autonomous,
            dt,
            motion: Some(MotionSample {
                elapsed,
                progress,
                wheel_targets: wheels,
            }),
        });
    }
}

/// Optional goal and action hooks for the runner's bounded mechanism sequence.
pub trait ScoringControls<R>: AutonomousControls<R> {
    type Target;
    fn set_target(robot: &mut R, target: Self::Target) -> Result<(), Self::Error>;
    fn at_target(robot: &R) -> bool;
    fn set_action(robot: &mut R, enabled: bool) -> Result<(), Self::Error>;
}

impl<D, C, T, X, P, A> MechanismRobot for Robot<D, C, T, X, P, A>
where
    P: ScoringControls<Self>,
    A: Telemetry<Self>,
    X: SubsystemList<P::SubsystemInput, P::Error>,
{
    type Target = P::Target;

    fn set_target(&mut self, target: Self::Target) -> Result<(), Self::Error> {
        P::set_target(self, target)
    }
    fn at_target(&self) -> bool {
        P::at_target(self)
    }
    fn set_action(&mut self, enabled: bool) -> Result<(), Self::Error> {
        P::set_action(self, enabled)
    }
}
