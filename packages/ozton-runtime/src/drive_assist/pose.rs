//! Adapt assisted VEX drives to generic pose motion.
#[cfg(feature = "logging")]
use std::time::Duration;
use std::time::Instant;

use ozton_drivetrain::{Drive, DriveMotionFault, Footprint, PoseDrive, model::Differential};
use ozton_localization::{Feedback, differential::DifferentialLocalization};
use ozton_math::{
    geometry::{DVec2, Pose},
    path::clearance::Obstacles,
    uom::si::f64::Velocity,
};
use ozton_tracking::{Tracking, TrackingSources};

use super::{AssistedDrivetrain, DriveAssist};

type AssistedTrackedDrive<'a, S, A> =
    AssistedDrivetrain<'a, Differential, Tracking<S>, DifferentialLocalization, A>;

impl<S, A> PoseDrive for AssistedTrackedDrive<'_, S, A>
where
    S: TrackingSources,
    A: DriveAssist<Differential, Tracking<S>, DifferentialLocalization>,
{
    type Error = DriveMotionFault;

    fn pose(&self) -> Pose {
        self.drivetrain().pose()
    }

    fn footprint(&self) -> Option<Footprint> {
        self.drivetrain().footprint()
    }

    fn rectangular_half_extents(&self) -> Option<DVec2> {
        self.drivetrain().rectangular_half_extents()
    }

    fn motion_obstacles(&self) -> Obstacles {
        self.drivetrain().motion_obstacles()
    }

    fn movable_obstacles(&self) -> Obstacles {
        self.drivetrain().movable_obstacles()
    }

    fn pose_feedback(&self) -> Result<Option<Feedback>, Self::Error> {
        self.drivetrain().pose_feedback()
    }

    fn position_covariance(&self) -> Option<glam::DMat2> {
        self.drivetrain().position_covariance()
    }

    fn velocity_uncertainty(&self) -> [f64; 3] {
        self.drivetrain().velocity_uncertainty()
    }

    fn drive_wheels(&mut self, speeds: [Velocity; 2]) -> Result<(), Self::Error> {
        self.drive_wheel_speeds(speeds).map_err(Into::into)
    }

    fn brake(&mut self) -> Result<(), Self::Error> {
        self.stop_now().map_err(DriveMotionFault::Brake)
    }

    fn pose_trusted(&self) -> bool {
        self.drivetrain().pose_trusted()
    }

    fn wait_until(&mut self, deadline: Instant) -> impl std::future::Future<Output = ()> {
        self.drivetrain_mut().wait_until(deadline)
    }

    fn motion_active(&self) -> bool {
        self.drivetrain().motion_active()
    }

    #[cfg(feature = "logging")]
    fn log_step(&mut self, elapsed: Duration, dt: Duration) {
        self.drivetrain_mut().log_step(elapsed, dt);
    }

    fn request_distance_reset(&mut self) {
        self.drivetrain_mut().request_distance_reset();
    }

    fn cancel_distance_reset(&mut self) {
        self.drivetrain_mut().cancel_distance_reset();
    }
}
