//! Short SI command helpers; every command uses the same cancellation and deadlines.
use std::time::Duration;

use ozton_math::geometry::Pose;

use super::{AutonError, AutonRobot, Command, Runner};
impl<R: AutonRobot> Runner<'_, R> {
    /// Explicit attestation of a surveyed starting/contact pose. Never infer it
    /// from an old recovered estimate or the power-on attitude.
    pub fn confirm_start(&mut self, pose: Pose) -> Result<(), AutonError<R::Error>> {
        self.robot().stop_drive().map_err(AutonError::Robot)?;
        self.robot().set_pose(pose).map_err(AutonError::Robot)
    }
    pub async fn drive_to_point(
        &mut self,
        target: [f64; 2],
        maximum_speed: f64,
        reverse: bool,
        timeout: Duration,
    ) -> Result<(), AutonError<R::Error>> {
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
        meters: f64,
        heading: f64,
        maximum_speed: f64,
        timeout: Duration,
    ) -> Result<(), AutonError<R::Error>> {
        self.run(
            Command::DriveDistance {
                meters,
                heading,
                maximum_speed,
            },
            timeout,
        )
        .await
    }
    pub async fn turn_to(
        &mut self,
        heading: f64,
        timeout: Duration,
    ) -> Result<(), AutonError<R::Error>> {
        self.run(Command::Turn { heading }, timeout).await
    }
    pub async fn wait(
        &mut self,
        duration: Duration,
        timeout: Duration,
    ) -> Result<(), AutonError<R::Error>> {
        self.run(Command::Wait(duration), timeout).await
    }
}
