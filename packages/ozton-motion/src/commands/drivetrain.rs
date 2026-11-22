//! Tracked-drive commands that borrow only the drivetrain.
use std::time::{Duration, Instant};

use ozton_drivetrain::{
    Drivetrain,
    model::{Differential, DriveEncoders, Tank, WheelSpeedError},
};
use ozton_localization::{
    Fault,
    differential::{DifferentialLocalization, update_localization},
};
use ozton_math::{
    geometry::{Pose, direction, wrap_heading},
    path::Path,
    uom::si::{
        angle::radian,
        f64::{Angle, Length, Velocity},
        velocity::meter_per_second,
    },
};
use ozton_runtime::clock::{CONTROL_PERIOD, CommandClock};
use ozton_tracking::sampling::{LocalizationLifecycle, SensorTracking};
use vexide::{
    competition::{self, CompetitionMode},
    smart::PortError,
};

use super::{
    Command, CommandError, DriveOptions, Feedback, Follower, MotionPlatform, Runner,
    TravelDirection, routine_settings::PATH_LOOKAHEAD,
};

type TrackedDrive = Drivetrain<Differential, SensorTracking, DifferentialLocalization>;
const MAX_CONTROL_GAP: Duration = Duration::from_millis(100);

#[derive(Clone, Copy, Debug)]
pub enum DriveMotionFault {
    Tracking(Fault),
    Output(PortError),
    Brake(PortError),
    InvalidCommand,
    MissingHeading,
    ContactFeedback,
    NoContact,
    Cancelled,
}

impl std::fmt::Display for DriveMotionFault {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for DriveMotionFault {}

impl From<WheelSpeedError> for DriveMotionFault {
    fn from(error: WheelSpeedError) -> Self {
        match error {
            WheelSpeedError::Port(port) => Self::Output(port),
            _ => Self::InvalidCommand,
        }
    }
}

impl MotionPlatform for TrackedDrive {
    type Error = DriveMotionFault;

    fn sample(&mut self) -> Result<Option<Feedback>, Self::Error> {
        self.poll_tracking(CompetitionMode::Autonomous)
            .map_err(DriveMotionFault::Tracking)
    }

    fn update_subsystems(&mut self, _: Duration) -> Result<(), Self::Error> {
        Ok(())
    }

    fn drive(&mut self, speeds: [Velocity; 2], _: Duration) -> Result<(), Self::Error> {
        self.drive_wheel_speeds(speeds).map_err(Into::into)
    }

    fn stop_drive(&mut self) -> Result<(), Self::Error> {
        self.stop_now().map_err(DriveMotionFault::Brake)
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.cancel_distance_reset();
        self.stop_drive()
    }

    fn set_pose(&mut self, pose: Pose) -> Result<(), Self::Error> {
        if self.localization.set_pose(pose) {
            Ok(())
        } else {
            Err(DriveMotionFault::InvalidCommand)
        }
    }

    fn pose_trusted(&self) -> bool {
        self.localization.pose_trusted()
    }

    fn request_distance_reset(&mut self) {
        self.localization.request_distance_reset();
    }

    fn cancel_distance_reset(&mut self) {
        self.localization.cancel_distance_reset();
    }
}

/// Geometry of a bracket that faces the goal center at contact.
/// All field and robot coordinates are ordered right, forward.
#[derive(Clone, Copy, Debug)]
pub struct GoalReset {
    pub center: [Length; 2],
    pub contact_radius: Length,
    pub bracket_from_origin: [Length; 2],
}

impl GoalReset {
    fn contact_pose(self, heading: Angle) -> Option<Pose> {
        let [right, forward] = self.bracket_from_origin;
        if !self.contact_radius.value.is_finite() || self.contact_radius.value <= 0.0 {
            return None;
        }
        let mut pose = Pose::new(self.center[0], self.center[1], heading);
        pose.position -= direction(heading) * self.contact_radius.value;
        pose.position = pose.transform_point(glam::DVec2::new(-right.value, -forward.value));
        pose.is_finite().then_some(pose)
    }
}

/// Standard motion methods for a drivetrain with a differential localizer.
#[allow(async_fn_in_trait)]
pub trait DrivetrainMotion {
    fn poll_tracking(&mut self, mode: CompetitionMode) -> Result<Option<Feedback>, Fault>;
    fn set_start_pose(&mut self, pose: Pose) -> Result<(), DriveMotionFault>;
    async fn drive_path(
        &mut self,
        path: &Path,
        travel: TravelDirection,
        timeout: Duration,
    ) -> Result<(), CommandError<DriveMotionFault>>;
    async fn try_distance_reset(
        &mut self,
        timeout: Duration,
    ) -> Result<bool, CommandError<DriveMotionFault>>;
    async fn reset_at_goal(
        &mut self,
        goal: GoalReset,
        speed: Velocity,
    ) -> Result<(), DriveMotionFault>;
}

impl DrivetrainMotion for TrackedDrive {
    fn poll_tracking(&mut self, mode: CompetitionMode) -> Result<Option<Feedback>, Fault> {
        if mode != CompetitionMode::Disabled {
            self.localization.set_competition_mode(mode);
        }
        if mode == CompetitionMode::Disabled {
            return Ok(None);
        }
        let positions =
            if LocalizationLifecycle::uses_drive_encoders(&self.localization, &self.tracking) {
                self.tracking_positions()
            } else {
                [None; 2]
            };
        let width = self.track_width();
        update_localization(&mut self.localization, &mut self.tracking, positions, width)
    }

    fn set_start_pose(&mut self, pose: Pose) -> Result<(), DriveMotionFault> {
        let heading = self
            .tracking
            .refresh_heading()
            .and_then(|sample| sample.heading);
        self.localization
            .register_start(pose, heading)
            .then_some(())
            .ok_or(DriveMotionFault::InvalidCommand)
    }

    async fn drive_path(
        &mut self,
        path: &Path,
        travel: TravelDirection,
        timeout: Duration,
    ) -> Result<(), CommandError<DriveMotionFault>> {
        let options = DriveOptions::for_differential(self, path.limits().wheel_speed)
            .ok_or(CommandError::Configuration)?;
        let mut runner = Runner::new(self, options, CONTROL_PERIOD, MAX_CONTROL_GAP, timeout)?;
        runner
            .run(
                Command::Follow {
                    path,
                    follower: Follower::PurePursuit {
                        lookahead: PATH_LOOKAHEAD,
                    },
                    reverse: matches!(travel, TravelDirection::Backward),
                    trigger: None,
                },
                timeout,
            )
            .await?;
        runner.finish_command()
    }

    async fn reset_at_goal(
        &mut self,
        goal: GoalReset,
        speed: Velocity,
    ) -> Result<(), DriveMotionFault> {
        if !speed.value.is_finite()
            || speed.value <= 0.0
            || goal.contact_pose(Angle::new::<radian>(0.0)).is_none()
        {
            return Err(DriveMotionFault::InvalidCommand);
        }

        let mut drive = StopOnDrop(Some(self));
        let drivetrain = drive
            .0
            .as_deref_mut()
            .ok_or(DriveMotionFault::InvalidCommand)?;
        let result = async {
            let mut clock =
                CommandClock::new(CONTROL_PERIOD).ok_or(DriveMotionFault::InvalidCommand)?;
            let started = Instant::now();
            let mut blocked_since = None;
            let start_heading = read_heading(drivetrain).ok_or(DriveMotionFault::MissingHeading)?;
            while started.elapsed() < Duration::from_millis(800) {
                if competition::mode() != CompetitionMode::Autonomous {
                    return Err(DriveMotionFault::Cancelled);
                }
                let heading = read_heading(drivetrain).ok_or(DriveMotionFault::MissingHeading)?;
                if wrap_heading(heading - start_heading).abs() > 0.15 {
                    return Err(DriveMotionFault::NoContact);
                }
                let [Some(left), Some(right)] = drivetrain.wheel_load() else {
                    return Err(DriveMotionFault::ContactFeedback);
                };
                let blocked = started.elapsed() >= Duration::from_millis(120)
                    && [left, right].iter().all(|side| {
                        side.mean_current.value >= 0.55 && side.fastest_wheel_speed.value <= 0.025
                    });
                if blocked {
                    let since = blocked_since.get_or_insert_with(Instant::now);
                    if since.elapsed() >= Duration::from_millis(60) {
                        return goal
                            .contact_pose(heading)
                            .ok_or(DriveMotionFault::InvalidCommand);
                    }
                } else {
                    blocked_since = None;
                }
                drivetrain.drive_wheel_speeds([speed, speed])?;
                if clock.tick().await > MAX_CONTROL_GAP {
                    return Err(DriveMotionFault::NoContact);
                }
            }
            Err(DriveMotionFault::NoContact)
        }
        .await;
        drive.brake()?;
        drop(drive);
        self.set_pose(result?)
    }

    async fn try_distance_reset(
        &mut self,
        timeout: Duration,
    ) -> Result<bool, CommandError<DriveMotionFault>> {
        // A wall reset holds the motors stationary. The runner still requires a positive speed.
        let options = DriveOptions::for_differential(self, Velocity::new::<meter_per_second>(1.0))
            .ok_or(CommandError::Configuration)?;
        let mut runner = Runner::new(
            self,
            options,
            CONTROL_PERIOD,
            MAX_CONTROL_GAP,
            timeout.saturating_add(MAX_CONTROL_GAP),
        )?;
        let reset = runner.try_distance_reset(timeout).await?;
        runner.finish_command()?;
        Ok(reset)
    }
}

fn read_heading(drivetrain: &mut TrackedDrive) -> Option<Angle> {
    let heading = drivetrain
        .tracking
        .refresh_heading()
        .and_then(|sample| sample.heading);
    drivetrain.localization.field_heading(heading)
}

struct StopOnDrop<'a>(Option<&'a mut TrackedDrive>);

impl StopOnDrop<'_> {
    fn brake(&mut self) -> Result<(), DriveMotionFault> {
        if let Some(drive) = self.0.as_deref_mut() {
            drive.stop_now().map_err(DriveMotionFault::Brake)?;
            self.0 = None;
        }
        Ok(())
    }
}

impl Drop for StopOnDrop<'_> {
    fn drop(&mut self) {
        let _ = self.brake();
    }
}

#[cfg(test)]
mod tests {
    use ozton_math::uom::si::length::meter;

    use super::*;

    #[test]
    fn goal_center_and_imu_heading_locate_the_offset_bracket() {
        let lengths = |values: [f64; 2]| values.map(Length::new::<meter>);
        let goal = GoalReset {
            center: lengths([2.0, 3.0]),
            contact_radius: Length::new::<meter>(0.5),
            bracket_from_origin: lengths([0.2, 0.1]),
        };
        let forward = goal.contact_pose(Angle::new::<radian>(0.0)).unwrap();
        assert!((forward.position - glam::DVec2::new(1.8, 2.4)).length() < 1e-12);
        let left = goal
            .contact_pose(Angle::new::<radian>(std::f64::consts::FRAC_PI_2))
            .unwrap();
        assert!((left.position - glam::DVec2::new(2.6, 2.8)).length() < 1e-12);
    }
}
