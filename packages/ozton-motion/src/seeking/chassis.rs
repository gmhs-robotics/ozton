//! Borrowed chassis commands for differential-drive autonomous routes.
use std::time::Duration;

use ozton_localization::LocalizationEstimate;
use ozton_math::{
    geometry::{DVec2, Pose},
    path::{Path, PathSample, clearance::RectangularClearance},
    uom::si::f64::{Angle, Length},
};

use super::{MotionExecution, PoseMotionError, RangeResetDrive, Seeking, WheelActuation};

type MotionResult<D, T = ()> = Result<T, PoseMotionError<<D as MotionExecution>::Error>>;

/// A borrowed drivetrain with a controller configuration. Awaiting a command
/// completes it before the next begins. No hardware or estimator is copied.
///
/// Bind once per autonomous route; use `using` to temporarily apply another
/// controller profile. Low-level `Seeking` commands remain available.
pub struct Chassis<'a, D> {
    drive: &'a mut D,
    controller: Seeking,
}

impl Seeking {
    /// Bind this motion configuration to a drivetrain for a sequence of commands.
    pub fn bind<D: MotionExecution>(self, drive: &mut D) -> Chassis<'_, D> {
        Chassis {
            drive,
            controller: self,
        }
    }
}

impl<D: MotionExecution> Chassis<'_, D> {
    /// Reborrow the chassis with a different controller profile.
    pub fn using(&mut self, controller: Seeking) -> Chassis<'_, D> {
        controller.bind(self.drive)
    }

    pub fn pose(&self) -> Pose {
        self.drive.localization().pose()
    }

    pub fn clearance(&self) -> MotionResult<D, Option<RectangularClearance>> {
        self.controller.clearance(self.drive)
    }

    pub fn prepare_path(
        &self,
        samples: &'static [PathSample],
        limits: [f64; 7],
    ) -> MotionResult<D, Path> {
        self.controller.prepare_path(self.drive, samples, limits)
    }

    pub async fn move_to_point(&mut self, point: impl Into<DVec2>) -> MotionResult<D> {
        self.controller.move_to_point(self.drive, point).await
    }

    pub async fn move_to_pose(&mut self, pose: Pose) -> MotionResult<D> {
        self.controller.move_to_pose(self.drive, pose).await
    }

    /// Turn, translate in a straight line, then acquire the final heading.
    /// Reject nominally obstructed moves before commanding the wheels.
    pub async fn move_to_pose_straight(&mut self, pose: Pose) -> MotionResult<D> {
        self.controller
            .move_to_pose_straight_checked(self.drive, pose)
            .await
    }

    pub async fn turn_to_heading(&mut self, heading: Angle) -> MotionResult<D> {
        self.controller.turn_to_heading(self.drive, heading).await
    }

    pub async fn turn_to_point(&mut self, point: impl Into<DVec2>) -> MotionResult<D> {
        self.controller.turn_to_point(self.drive, point).await
    }

    /// Signed distance PID. A registered physical wall contact is automatically
    /// released by a validated inward departure, not by an arbitrary pose reset.
    pub async fn drive_distance(&mut self, distance: Length) -> MotionResult<D> {
        let contact = self.drive.localization().wall_contact();
        if let Some(wall) = contact {
            self.controller
                .drive_distance_from_contact(self.drive, distance, wall)
                .await
        } else {
            self.controller.drive_distance(self.drive, distance).await
        }
    }

    pub async fn drive_arc(&mut self, radius: Length, sweep: Angle) -> MotionResult<D> {
        self.controller.drive_arc(self.drive, radius, sweep).await
    }

    pub async fn follow_path(&mut self, path: &Path) -> MotionResult<D> {
        self.controller.follow(self.drive, path).await
    }

    /// Follow waypoints with straight legs, chaining collinear transit points.
    pub async fn follow_waypoints(
        &mut self,
        points: &[DVec2],
        handoff_radius: Length,
    ) -> MotionResult<D> {
        self.controller
            .move_through_points_straight(self.drive, points, handoff_radius)
            .await
    }

    /// Chain transit points where corner cutting is acceptable.
    pub async fn move_through_points(
        &mut self,
        points: &[DVec2],
        handoff_radius: Length,
    ) -> MotionResult<D> {
        self.controller
            .move_through_points(self.drive, points, handoff_radius)
            .await
    }

    pub async fn follow_poses(&mut self, poses: &[Pose]) -> MotionResult<D> {
        self.controller.follow_poses(self.drive, poses).await
    }
}

impl<D: RangeResetDrive> Chassis<'_, D> {
    /// Request a stationary localization correction using the available ranges.
    pub async fn try_distance_reset(&mut self, timeout: Duration) -> MotionResult<D, bool> {
        self.controller.try_distance_reset(self.drive, timeout).await
    }
}

impl<D: MotionExecution + WheelActuation> Chassis<'_, D>
where
    D::Error: From<D::WheelError>,
{
    pub async fn swing_to_heading(&mut self, heading: Angle) -> MotionResult<D> {
        self.controller.swing_to_heading(self.drive, heading).await
    }
}
