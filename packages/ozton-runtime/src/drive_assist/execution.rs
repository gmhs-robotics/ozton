//! Adapt assisted VEX drives to generic pose motion.
#[cfg(feature = "logging")]
use std::time::Duration;
use std::time::Instant;

use ozton_drivetrain::{
    Drive, DriveMotionFault, Footprint, MotionExecution, RangeResetDrive,
    model::{Differential, MotionGeometryFor},
};
use ozton_localization::{Localization, field::Boundary};
use ozton_math::uom::si::f64::Velocity;
use ozton_tracking::{HasDriveWheels, HasRange, Tracking, TrackingSources};

use super::{AssistedDrivetrain, DriveAssist};

type AssistedTrackedDrive<'a, G, S, B, A> =
    AssistedDrivetrain<'a, Differential<G>, Tracking<S>, Localization<B>, A>;

impl<G, S, B, A> MotionExecution for AssistedTrackedDrive<'_, G, S, B, A>
where
    B: Boundary,
    G: MotionGeometryFor<B>,
    S: TrackingSources,
    Tracking<S>: HasDriveWheels,
    A: DriveAssist<Differential<G>, Tracking<S>, Localization<B>>,
{
    type Error = DriveMotionFault;

    fn footprint(&self) -> Option<Footprint> {
        self.drivetrain().footprint()
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

}

impl<G, S, B, A> RangeResetDrive for AssistedTrackedDrive<'_, G, S, B, A>
where
    B: Boundary,
    G: MotionGeometryFor<B>,
    S: TrackingSources,
    Tracking<S>: HasDriveWheels + HasRange,
    A: DriveAssist<Differential<G>, Tracking<S>, Localization<B>>,
{
}
