//! Route commands on a fully assembled competition robot.

use std::time::Duration;

use ozton_math::{
    geometry::{DVec2, Pose},
    path::{Limits, Path, Waypoint, follower::Follower},
    uom::si::f64::{Angle, Length, Velocity},
};
use ozton_runtime::{
    Robot,
    competition::{Autonomous, Program, ProgramDrive, RouteSet},
    diagnostics::Telemetry,
};

use super::{
    Alignment, Command, CommandError, MotionControls, MotionPlatform, Runner,
    routine_settings::{
        AUTON_SPEED, PATH_ACCELERATION, PATH_DECELERATION, PATH_LATERAL_ACCELERATION,
        PATH_LOOKAHEAD,
    },
};

#[derive(Clone, Copy)]
pub enum TravelDirection {
    Forward,
    Backward,
}

/// Closed-loop commands using the current route's deadline and installed sensors.
#[allow(async_fn_in_trait)]
pub trait RobotMotion: Sized {
    type Error;

    fn confirm_start(&mut self, pose: Pose) -> Result<(), CommandError<Self::Error>>;

    async fn drive(
        &mut self,
        direction: TravelDirection,
        distance: Length,
    ) -> Result<(), CommandError<Self::Error>>;

    async fn turn_to(&mut self, heading: Angle) -> Result<(), CommandError<Self::Error>>;

    async fn turn_toward(&mut self, target: DVec2) -> Result<(), CommandError<Self::Error>>;

    async fn drive_to(&mut self, target: DVec2) -> Result<(), CommandError<Self::Error>> {
        self.drive_to_with(target, TravelDirection::Forward).await
    }

    async fn drive_to_with(
        &mut self,
        target: DVec2,
        direction: TravelDirection,
    ) -> Result<(), CommandError<Self::Error>>;

    /// Profile a PATH.JERRYIO export using the robot's motion limits.
    fn load_path(&self, export: &[u8]) -> Result<Path, CommandError<Self::Error>> {
        let points =
            ozton_path_io::jerryio::parse(export).map_err(|_| CommandError::InvalidPath)?;
        self.profile_path(&points)
    }

    fn profile_path(&self, waypoints: &[Waypoint]) -> Result<Path, CommandError<Self::Error>>;

    async fn follow_path(
        &mut self,
        path: &Path,
        direction: TravelDirection,
    ) -> Result<(), CommandError<Self::Error>>;

    /// Follow a prepared path with the selected Ozton algorithm and tuning.
    async fn follow_path_with(
        &mut self,
        path: &Path,
        direction: TravelDirection,
        follower: Follower,
    ) -> Result<(), CommandError<Self::Error>>;

    async fn wait(&mut self, duration: Duration) -> Result<(), CommandError<Self::Error>> {
        self.run_command(Command::Wait(duration)).await
    }

    /// A clean alignment timeout leaves the drive braked and returns false.
    async fn try_align(
        &mut self,
        alignment: Alignment,
        timeout: Duration,
    ) -> Result<bool, CommandError<Self::Error>>;

    async fn run_command(
        &mut self,
        command: Command<'_, Self>,
    ) -> Result<(), CommandError<Self::Error>>;
}

impl<D, C, T, X, P, R, S> RobotMotion for Robot<D, C, T, X, P, Autonomous<R, S>>
where
    D: ProgramDrive<AutonCommand = [Velocity; 2]>,
    X: ozton_runtime::competition::Mechanism<
            <P as MotionControls<Robot<D, C, T, X, P, Autonomous<R, S>>>>::Error,
        >,
    P: MotionControls<Self> + Program<Self, D, Error = <P as MotionControls<Self>>::Error>,
    R: RouteSet,
    Autonomous<R, S>: Telemetry<Self>,
{
    type Error = <P as MotionControls<Self>>::Error;

    fn confirm_start(&mut self, pose: Pose) -> Result<(), CommandError<Self::Error>> {
        super::MotionPlatform::stop_drive(self).map_err(CommandError::StopFailed)?;
        P::set_pose(self, pose).map_err(CommandError::Robot)
    }

    async fn drive(
        &mut self,
        direction: TravelDirection,
        distance: Length,
    ) -> Result<(), CommandError<Self::Error>> {
        if !distance.value.is_finite() || distance.value < 0.0 {
            return Err(CommandError::Configuration);
        }
        let options = P::drive_options(self).ok_or(CommandError::Configuration)?;
        let meters = match direction {
            TravelDirection::Forward => distance,
            TravelDirection::Backward => -distance,
        };
        self.run_command(Command::DriveDistance {
            meters,
            heading: None,
            maximum_speed: if AUTON_SPEED <= options.maximum_wheel_speed {
                AUTON_SPEED
            } else {
                options.maximum_wheel_speed
            },
        })
        .await
    }

    async fn turn_to(&mut self, heading: Angle) -> Result<(), CommandError<Self::Error>> {
        self.run_command(Command::Turn { heading }).await
    }

    async fn turn_toward(&mut self, target: DVec2) -> Result<(), CommandError<Self::Error>> {
        self.run_command(Command::TurnToPoint { target }).await
    }

    async fn drive_to_with(
        &mut self,
        target: DVec2,
        direction: TravelDirection,
    ) -> Result<(), CommandError<Self::Error>> {
        let options = P::drive_options(self).ok_or(CommandError::Configuration)?;
        self.run_command(Command::DriveToPoint {
            target,
            reverse: matches!(direction, TravelDirection::Backward),
            maximum_speed: options.maximum_wheel_speed,
        })
        .await
    }

    fn profile_path(&self, waypoints: &[Waypoint]) -> Result<Path, CommandError<Self::Error>> {
        let options = P::drive_options(self).ok_or(CommandError::Configuration)?;
        let speed = if AUTON_SPEED <= options.maximum_wheel_speed {
            AUTON_SPEED
        } else {
            options.maximum_wheel_speed
        };
        Path::new(
            waypoints,
            Limits {
                track_width: options.track_width,
                linear_speed: speed,
                wheel_speed: options.maximum_wheel_speed,
                acceleration: PATH_ACCELERATION,
                deceleration: PATH_DECELERATION,
                lateral_acceleration: PATH_LATERAL_ACCELERATION,
            },
        )
        .map_err(|_| CommandError::InvalidPath)
    }

    async fn follow_path(
        &mut self,
        path: &Path,
        direction: TravelDirection,
    ) -> Result<(), CommandError<Self::Error>> {
        self.follow_path_with(
            path,
            direction,
            Follower::PurePursuit {
                lookahead: PATH_LOOKAHEAD,
            },
        )
        .await
    }

    async fn follow_path_with(
        &mut self,
        path: &Path,
        direction: TravelDirection,
        follower: Follower,
    ) -> Result<(), CommandError<Self::Error>> {
        self.run_command(Command::Follow {
            path,
            follower,
            reverse: matches!(direction, TravelDirection::Backward),
            trigger: None,
        })
        .await
    }

    async fn try_align(
        &mut self,
        alignment: Alignment,
        timeout: Duration,
    ) -> Result<bool, CommandError<Self::Error>> {
        let result = async {
            let (mut runner, stage) = route_runner(self)?;
            let aligned = runner.try_align(alignment, timeout.min(stage)).await?;
            runner.finish_command()?;
            Ok(aligned)
        }
        .await;
        if result.is_err() {
            self.autonomous.cancel_motion();
        }
        result
    }

    async fn run_command(
        &mut self,
        command: Command<'_, Self>,
    ) -> Result<(), CommandError<Self::Error>> {
        let result = async {
            let (mut runner, stage) = route_runner(self)?;
            runner.run(command, stage).await?;
            runner.finish_command()
        }
        .await;
        if result.is_err() {
            self.autonomous.cancel_motion();
        }
        result
    }
}

type RouteRobot<D, C, T, X, P, R, S> = Robot<D, C, T, X, P, Autonomous<R, S>>;
type RouteRunner<'a, D, C, T, X, P, R, S> = Result<
    (Runner<'a, RouteRobot<D, C, T, X, P, R, S>>, Duration),
    CommandError<<RouteRobot<D, C, T, X, P, R, S> as MotionPlatform>::Error>,
>;

fn route_runner<D, C, T, X, P, R, S>(
    robot: &mut RouteRobot<D, C, T, X, P, R, S>,
) -> RouteRunner<'_, D, C, T, X, P, R, S>
where
    P: MotionControls<RouteRobot<D, C, T, X, P, R, S>>,
    D: ProgramDrive<AutonCommand = [Velocity; 2]>,
    X: ozton_runtime::competition::Mechanism<
            <P as MotionControls<RouteRobot<D, C, T, X, P, R, S>>>::Error,
        >,
    P: Program<
            RouteRobot<D, C, T, X, P, R, S>,
            D,
            Error = <P as MotionControls<RouteRobot<D, C, T, X, P, R, S>>>::Error,
        >,
    R: RouteSet,
    Autonomous<R, S>: Telemetry<RouteRobot<D, C, T, X, P, R, S>>,
{
    let started = robot
        .autonomous
        .started_at()
        .ok_or(CommandError::Cancelled)?;
    let limits = P::session_limits(robot);
    if !limits.valid() {
        return Err(CommandError::Configuration);
    }
    let remaining = limits.total.saturating_sub(started.elapsed());
    let period = P::motion_period(robot);
    if remaining < period {
        return Err(CommandError::RouteTimeout);
    }
    let options = P::drive_options(robot).ok_or(CommandError::Configuration)?;
    let gap = P::maximum_motion_gap(robot);
    let runner = Runner::new(robot, options, period, gap, remaining)?;
    Ok((runner, limits.stage))
}
