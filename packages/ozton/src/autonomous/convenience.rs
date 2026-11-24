//! Short SI command helpers; every command uses the same cancellation and deadlines.
use std::time::Duration;

use ozton_math::{
    geometry::{DVec2, Pose},
    uom::si::f64::{Angle, Length, Velocity},
};

use super::{Command, CommandError, MotionPlatform, Runner};
impl<R: MotionPlatform> Runner<'_, R> {
    /// Explicit attestation of a surveyed starting/contact pose. Never infer it
    /// from an old recovered estimate or the power-on attitude.
    pub fn confirm_start(&mut self, pose: Pose) -> Result<(), CommandError<R::Error>> {
        self.robot().stop_drive().map_err(CommandError::Robot)?;
        self.robot().set_pose(pose).map_err(CommandError::Robot)
    }
    pub async fn drive_to_point(
        &mut self,
        target: DVec2,
        maximum_speed: Velocity,
        reverse: bool,
        timeout: Duration,
    ) -> Result<(), CommandError<R::Error>> {
        self.run(
            Command::DriveToPoint {
                target,
                reverse,
                maximum_speed,
            },
            timeout,
        )
        .await
    }
    pub async fn drive_distance(
        &mut self,
        meters: Length,
        heading: Angle,
        maximum_speed: Velocity,
        timeout: Duration,
    ) -> Result<(), CommandError<R::Error>> {
        self.run(
            Command::DriveDistance {
                meters,
                heading: Some(heading),
                maximum_speed,
            },
            timeout,
        )
        .await
    }
    pub async fn turn_to(
        &mut self,
        heading: Angle,
        timeout: Duration,
    ) -> Result<(), CommandError<R::Error>> {
        self.run(Command::Turn { heading }, timeout).await
    }
    pub async fn wait(
        &mut self,
        duration: Duration,
        timeout: Duration,
    ) -> Result<(), CommandError<R::Error>> {
        self.run(Command::Wait(duration), timeout).await
    }
}
