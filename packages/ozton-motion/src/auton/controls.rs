//! Motion hooks for an assembled Ozton robot.
use std::time::Duration;

use ozton_drivetrain::model::ProgramDrive;
use ozton_math::{
    geometry::Pose,
    uom::si::f64::{Length, Velocity},
};
use ozton_runtime::{
    Robot,
    competition::Program,
    diagnostics::{MotionSample, Sample, Telemetry},
};
use vexide::competition::CompetitionMode;

use super::{AutonRobot, Feedback, MechanismRobot};

/// Connect the installed control policy to the autonomous runner. Each hook
/// receives the assembled robot so feedback and outputs can use its concrete
/// devices. The runner owns timing, cancellation, retries and motion algorithms.
pub trait AutonomousControls<R> {
    type Error;

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
    /// Clear autonomous state associated with the current motion session.
    fn clear_motion(_robot: &mut R) {}
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
    fn pause(_robot: &mut R) -> Result<(), Self::Error> {
        Ok(())
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
    D: ProgramDrive<AutonCommand = [Velocity; 2]>,
    P: AutonomousControls<Self> + Program<Self, D, Error = <P as AutonomousControls<Self>>::Error>,
    A: Telemetry<Self>,
{
    type Error = <P as AutonomousControls<Self>>::Error;

    fn sample(&mut self) -> Result<Option<Feedback>, Self::Error> {
        P::sample(self)
    }
    fn update_subsystems(&mut self, dt: Duration) -> Result<(), Self::Error> {
        P::update_mechanisms(self, CompetitionMode::Autonomous, dt)
    }
    fn drive(&mut self, wheel_speeds: [Velocity; 2], dt: Duration) -> Result<(), Self::Error> {
        self.drivetrain
            .apply_auton(wheel_speeds, dt)
            .map_err(|error| P::drive_error(error, false))
    }
    fn stop_drive(&mut self) -> Result<(), Self::Error> {
        self.drivetrain
            .stop()
            .map_err(|error| P::drive_error(error, true))
    }
    fn stop(&mut self) -> Result<(), Self::Error> {
        P::clear_motion(self);
        self.stop_drive().and(P::stop_mechanisms(self))
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
        self.stop_drive()
            .and(P::pause_mechanisms(self))
            .and(P::pause(self))
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
    D: ProgramDrive<AutonCommand = [Velocity; 2]>,
    P: ScoringControls<Self> + Program<Self, D, Error = <P as AutonomousControls<Self>>::Error>,
    A: Telemetry<Self>,
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

impl<C, T, X, P, A> AutonomousControls<Robot<super::ConfiguredChassis, C, T, X, P, A>> for P
where
    P: Program<Robot<super::ConfiguredChassis, C, T, X, P, A>, super::ConfiguredChassis>,
{
    type Error = <P as Program<
        Robot<super::ConfiguredChassis, C, T, X, P, A>,
        super::ConfiguredChassis,
    >>::Error;

    fn drive_options(
        robot: &Robot<super::ConfiguredChassis, C, T, X, P, A>,
    ) -> Option<super::DriveOptions> {
        Some(robot.drivetrain.drive_options())
    }

    fn session_limits(
        robot: &Robot<super::ConfiguredChassis, C, T, X, P, A>,
    ) -> super::routine_settings::SessionLimits {
        let total = robot.drivetrain.settings.route_timeout;
        super::routine_settings::SessionLimits {
            total,
            stage: total.min(Duration::from_secs(5)),
        }
    }

    fn motion_period(_: &Robot<super::ConfiguredChassis, C, T, X, P, A>) -> Duration {
        P::CONTROL_PERIOD
    }

    fn maximum_motion_gap(robot: &Robot<super::ConfiguredChassis, C, T, X, P, A>) -> Duration {
        robot.drivetrain.settings.max_control_gap
    }

    fn sample(
        robot: &mut Robot<super::ConfiguredChassis, C, T, X, P, A>,
    ) -> Result<Option<Feedback>, Self::Error> {
        let occluded = P::range_occlusion(robot);
        robot
            .drivetrain
            .sample(occluded)
            .map_err(|error| P::drive_error(error, false))
    }

    fn set_pose(
        robot: &mut Robot<super::ConfiguredChassis, C, T, X, P, A>,
        pose: Pose,
    ) -> Result<(), Self::Error> {
        robot
            .drivetrain
            .set_pose(pose)
            .map_err(|error| P::drive_error(error, true))
    }

    fn request_distance_reset(robot: &mut Robot<super::ConfiguredChassis, C, T, X, P, A>) {
        robot.drivetrain.request_distance_reset();
    }

    fn cancel_distance_reset(robot: &mut Robot<super::ConfiguredChassis, C, T, X, P, A>) {
        robot.drivetrain.cancel_distance_reset();
    }

    fn pose_trusted(robot: &Robot<super::ConfiguredChassis, C, T, X, P, A>) -> bool {
        robot.drivetrain.chassis().pose_trusted()
    }

    fn log_step(
        robot: &mut Robot<super::ConfiguredChassis, C, T, X, P, A>,
        elapsed: Duration,
        dt: Duration,
        progress: Length,
        wheels: [Velocity; 2],
    ) {
        robot.drivetrain.log_step(elapsed, dt, progress, wheels);
    }
}
