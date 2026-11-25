//! Adapt assisted VEX drives to generic pose motion.
#[cfg(feature = "logging")]
use std::time::Duration;
use std::time::Instant;

use ozton_drivetrain::{
    BrakeMode, BrakeMotors, Drive, DriveMotionFault, Footprint, MotorAddress, PoseDrive,
    RangeResetDrive, WheelCommand, WheelCommandDrive,
    model::{Differential, MotionGeometryFor},
};
use ozton_localization::{Feedback, differential::DifferentialLocalization, field::Boundary};
use ozton_math::{
    geometry::{DVec2, Pose},
    path::clearance::{Obstacles, WallContact},
    uom::si::f64::Velocity,
};
use ozton_tracking::{HasDriveWheels, HasRange, Tracking, TrackingSources};

use super::{AssistedDrivetrain, DriveAssist};

type AssistedTrackedDrive<'a, G, S, B, A> =
    AssistedDrivetrain<'a, Differential<G>, Tracking<S>, DifferentialLocalization<B>, A>;

impl<G, S, B, A> PoseDrive for AssistedTrackedDrive<'_, G, S, B, A>
where
    B: Boundary,
    G: MotionGeometryFor<B>,
    S: TrackingSources,
    Tracking<S>: HasDriveWheels,
    A: DriveAssist<Differential<G>, Tracking<S>, DifferentialLocalization<B>>,
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

    fn position_covariance(&self) -> Option<ozton_localization::PositionCovariance> {
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

    fn emergency_brake(&mut self) -> Result<(), Self::Error> {
        self.drivetrain_mut().emergency_brake()
    }

    fn wall_contact(&self) -> Option<WallContact> {
        self.drivetrain().wall_contact()
    }

    fn release_wall_contact(&mut self) {
        self.drivetrain_mut().release_wall_contact();
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

    fn cancel_distance_reset(&mut self) {
        self.drivetrain_mut().cancel_distance_reset();
    }
}

impl<G, S, B, A> WheelCommandDrive for AssistedTrackedDrive<'_, G, S, B, A>
where
    B: Boundary,
    G: MotionGeometryFor<B>,
    S: TrackingSources,
    Tracking<S>: HasDriveWheels,
    A: DriveAssist<Differential<G>, Tracking<S>, DifferentialLocalization<B>>,
{
    fn drive_wheel_commands(&mut self, commands: [WheelCommand; 2]) -> Result<(), Self::Error> {
        self.command_wheels(commands).map_err(Into::into)
    }
}

impl<G, S, B, A> BrakeMotors for AssistedTrackedDrive<'_, G, S, B, A>
where
    B: Boundary,
    G: MotionGeometryFor<B>,
    S: TrackingSources,
    Tracking<S>: HasDriveWheels,
    A: DriveAssist<Differential<G>, Tracking<S>, DifferentialLocalization<B>>,
{
    type BrakeError = vexide::smart::PortError;

    fn brake_motors(
        &mut self,
        select: impl FnMut(MotorAddress) -> Option<BrakeMode>,
    ) -> Result<(), Self::BrakeError> {
        self.drivetrain_mut().brake_motors(select)
    }
}

impl<G, S, B, A> RangeResetDrive for AssistedTrackedDrive<'_, G, S, B, A>
where
    B: Boundary,
    G: MotionGeometryFor<B>,
    S: TrackingSources,
    Tracking<S>: HasDriveWheels + HasRange,
    A: DriveAssist<Differential<G>, Tracking<S>, DifferentialLocalization<B>>,
{
    fn request_distance_reset(&mut self) {
        <_ as RangeResetDrive>::request_distance_reset(self.drivetrain_mut());
    }
}
