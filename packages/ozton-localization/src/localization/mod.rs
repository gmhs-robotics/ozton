//! Convert timestamped tracking observations into robot pose and motion.
use std::time::Duration;

#[cfg(feature = "logging")]
use ozton_diagnostics::log_event;
use ozton_diagnostics::log_row;
use ozton_math::{
    geometry::{DVec2, Pose},
    localization::observation::{
        MAX_RANGES, SensorSample, TracksForwardTravel, TracksHeading, TracksPosition,
        TracksVelocity,
    },
    path::clearance::{Footprint, Obstacles, WallContact, WallContactConstraint},
    uom::si::{
        angular_velocity::radian_per_second,
        f64::{Angle, AngularVelocity, Length, Velocity},
        velocity::meter_per_second,
    },
};
pub use wheel_odometry::DriveWheels as DriveEncoderUse;
use wheel_odometry::WheelOdometry;
mod wheel_odometry;

use crate::{
    Feedback, RangeFeedback,
    field::{Boundary, FieldBoundary},
    localizer::{DistanceReset, Estimator, Fault, Localizer, PoseTrust},
};

mod builder;
mod observations;
#[cfg(feature = "logging")]
mod telemetry;
pub use builder::{LocalizationBuilder, OdometrySettings, RangeRequired};

/// Pose estimator state. Accepts observations without owning or reading hardware.
pub struct Localization<B = ()> {
    localizer: Localizer<'static>,
    boundary: B,
    motion_obstacles: Obstacles,
    movable_obstacles: Obstacles,
    wheels: WheelOdometry,
    observations: observations::Observations,
    drive: Option<DriveEncoderUse>,
    use_imu: bool,
    occluded: u8,
    last_reset: DistanceReset,
    feedback: Result<Option<Feedback>, Fault>,
    #[cfg(feature = "logging")]
    fault: Option<Fault>,
    #[cfg(feature = "logging")]
    telemetry: telemetry::Telemetry,
}

impl Localization {
    pub fn builder() -> LocalizationBuilder {
        LocalizationBuilder::new()
    }
}

impl Localization<FieldBoundary> {
    /// Compute the tracked pose from the chassis's lower-left bounding corner,
    /// measured inward from the field's left and rear walls.
    pub fn pose_from_corner(
        &self,
        footprint: Footprint,
        offset: DVec2,
        heading: Angle,
    ) -> Pose {
        let (min, _) = footprint.bounds(Pose::new(DVec2::ZERO, heading));
        Pose::new(-self.boundary.half_extents() + offset - min, heading)
    }

    /// Register the surveyed pose of a chassis flush against a real wall.
    /// The caller verifies physical contact; a virtual motion exclusion is
    /// not a valid contact observation.
    pub fn set_pose_at_contact(
        &mut self,
        pose: Pose,
        footprint: Footprint,
        wall: WallContact,
        tangential_sigma: Length,
    ) -> bool {
        let Some(contact) = WallContactConstraint::new(
            wall,
            footprint,
            self.boundary.half_extents(),
            pose,
            tangential_sigma,
        ) else {
            return false;
        };
        if !self.localizer.set_pose_at_contact(pose, contact) {
            return false;
        }
        self.last_reset = DistanceReset::Idle;
        self.feedback = Ok(None);
        self.wheels.observe(&[]);
        self.log_pose_registration(pose, true);
        true
    }

    /// Update containment dimensions without removing the boundary.
    /// The new dimensions must already have been validated.
    pub fn set_motion_bounds(&mut self, boundary: FieldBoundary) {
        self.boundary = boundary;
    }
}

impl<B: Boundary> Localization<B> {
    /// The validated boundary state, or unit for an unbounded estimator.
    pub fn boundary(&self) -> B {
        self.boundary
    }

    pub fn wall_contact(&self) -> Option<WallContact> {
        self.localizer.wall_contact()
    }

    pub fn release_wall_contact(&mut self) {
        self.localizer.release_wall_contact();
    }

    /// Fixed footprints projected from the unified field geometry.
    pub fn motion_obstacles(&self) -> Obstacles {
        self.motion_obstacles
    }

    /// Movable footprints projected from the same faces used by localization.
    pub fn movable_obstacles(&self) -> Obstacles {
        self.movable_obstacles
    }

    /// Keep raw tracking and logging active while suspending all external corrections.
    pub fn set_corrections_enabled(&mut self, enabled: bool) {
        self.localizer.set_corrections_enabled(enabled);
        self.last_reset = DistanceReset::Idle;
    }

    pub fn pause(&mut self) {
        self.feedback = Ok(None);
        self.localizer.invalidate_pose();
        self.wheels.observe(&[]);
    }

    pub fn pose(&self) -> Pose {
        self.localizer.pose()
    }

    pub fn pose_trusted(&self) -> bool {
        self.localizer.pose_trust() == PoseTrust::Trusted
    }

    /// Sets the field pose and clears the odometry baselines. IMU observations
    /// contribute heading changes from the next baseline sample.
    pub fn set_pose(&mut self, pose: Pose) -> bool {
        if !self.localizer.set_pose(pose) {
            return false;
        }
        self.last_reset = DistanceReset::Idle;
        self.feedback = Ok(None);
        self.wheels.observe(&[]);
        self.log_pose_registration(pose, false);
        true
    }

    fn log_pose_registration(&self, pose: Pose, wall_contact: bool) {
        let [width, length] = self
            .boundary
            .bounds()
            .map_or([f64::NAN; 2], |b| [2.0 * b.x, 2.0 * b.y]);
        log_row!(
            Info,
            "LOCALIZATION_REGISTER",
            "width_m={width:.6},length_m={length:.6},start_x_m={:.6},start_y_m={:.6},start_heading_rad={:.6},wall_contact={wall_contact}",
            pose.position.x,
            pose.position.y,
            pose.heading.value
        );
    }

    pub fn set_estimator(&mut self, estimator: Estimator, shadow: bool) -> bool {
        if !self.localizer.set_estimator(estimator, shadow) {
            return false;
        }
        if matches!(estimator, Estimator::Odometry) {
            self.localizer.disable_range_inspection();
        }
        log_row!(
            Info,
            "LOCALIZATION_ESTIMATOR",
            "mcl={},shadow={shadow}",
            matches!(estimator, Estimator::Mcl(_))
        );
        true
    }

    pub fn set_dsr_config(&mut self, config: crate::DsrConfig) -> bool {
        if !self.localizer.set_dsr_config(config) {
            return false;
        }
        self.last_reset = DistanceReset::Idle;
        true
    }

    pub fn request_distance_reset(&mut self) {
        self.last_reset = DistanceReset::Idle;
        if let Ok(Some(feedback)) = &mut self.feedback {
            feedback.reset_accepted = false;
        }
        self.localizer.request_distance_reset();
    }

    pub fn cancel_distance_reset(&mut self) {
        self.localizer.cancel_distance_reset();
    }

    pub fn distance_reset_axes(&self) -> u8 {
        match self.last_reset {
            DistanceReset::Applied { axes } => axes,
            _ => 0,
        }
    }

    pub fn set_occluded(&mut self, mask: u8) {
        self.occluded = mask;
    }

    pub fn occluded(&self) -> u8 {
        self.occluded
    }

    pub fn uses_drive_encoders(&self) -> bool {
        self.drive.is_some()
    }

    pub fn position_covariance(&self) -> Option<glam::DMat2> {
        self.localizer.position_covariance()
    }

    pub fn velocity_uncertainty(&self) -> [f64; 3] {
        self.localizer.velocity_uncertainty()
    }

    pub fn feedback(&self) -> Result<Option<Feedback>, Fault> {
        self.feedback
    }

    pub fn update(
        &mut self,
        sample: SensorSample<'_>,
        track_width: Option<Length>,
    ) -> Result<Option<Feedback>, Fault> {
        self.feedback = self.update_sample(sample, track_width);
        self.feedback
    }

    fn update_sample(
        &mut self,
        sample: SensorSample<'_>,
        track_width: Option<Length>,
    ) -> Result<Option<Feedback>, Fault> {
        let (snapshot, wheels, ranges, mut rates) = self.observations.read(sample);
        if self.drive.is_none() || self.wheels.has_forward_projection() {
            rates.forward_rate = None;
        }
        if !self.use_imu {
            rates.yaw_rate = None;
            rates.acceleration = None;
        }
        self.wheels.observe(wheels);
        let update = match self.localizer.update_with_motion(
            snapshot,
            &ranges,
            rates,
            |before, after, _, _| {
                Ok(self
                    .wheels
                    .delta(before, after, self.drive, track_width, self.use_imu))
            },
        ) {
            Ok(update) => {
                #[cfg(feature = "logging")]
                if self.fault.take().is_some() {
                    log_event!(Info, "tracking", "Localization feedback recovered");
                }
                update
            }
            Err(Fault::Initializing) => {
                log_row!(
                    Trace,
                    "TRACKING_UPDATE",
                    "status=initializing,snapshot_ms={}",
                    snapshot.time.as_millis()
                );
                return Ok(None);
            }
            Err(fault) => {
                #[cfg(feature = "logging")]
                if self.fault.replace(fault) != Some(fault) {
                    let reason = match fault {
                        Fault::Initializing => "Waiting for a second tracking observation",
                        Fault::Time => "Tracking timestamp repeated or exceeded maximum gap",
                        Fault::Forward => "Forward travel unavailable",
                        Fault::Heading => "Heading unavailable",
                        Fault::Lateral => "Lateral travel unavailable",
                        Fault::InvalidEstimate => "Estimator produced an invalid pose",
                    };
                    log_event!(
                        Warn,
                        "tracking",
                        "{reason} at sample {} ms",
                        snapshot.time.as_millis()
                    );
                }
                log_row!(
                    Trace,
                    "TRACKING_UPDATE",
                    "status=fault,fault={fault:?},snapshot_ms={}",
                    snapshot.time.as_millis()
                );
                return Err(fault);
            }
        };

        #[cfg(feature = "logging")]
        {
            let _span = ozton_diagnostics::profile::Span::new(
                ozton_diagnostics::profile::Stage::LocalizationLog,
            );
            self.telemetry.record(
                &update,
                &self.localizer,
                wheels,
                &ranges,
                self.occluded,
                rates,
            );
        }

        if update.reset != DistanceReset::Idle {
            self.last_reset = update.reset;
        }
        let mut relative_ranges = [None; MAX_RANGES];
        for (index, reading) in ranges.into_iter().enumerate() {
            if let Some(reading) = reading
                && let Some(config) = self.localizer.range_config(index)
                && let Ok(distance) = config.relative_distance(reading, snapshot.time)
            {
                relative_ranges[index] = Some(RangeFeedback {
                    distance,
                    timestamp: reading.timestamp,
                });
            }
        }
        Ok(Some(Feedback {
            time: snapshot.time,
            pose: update.pose,
            forward_travel: ozton_math::qty!((self.localizer.forward_travel()) m),
            forward_velocity: Velocity::new::<meter_per_second>(update.forward_velocity),
            lateral_velocity: Velocity::new::<meter_per_second>(update.lateral_velocity),
            angular_velocity: AngularVelocity::new::<radian_per_second>(update.angular_velocity),
            ranges: relative_ranges,
            reset_accepted: update.distance_reset,
        }))
    }
}

impl<B: Boundary> TracksPosition for Localization<B> {
    fn position(&self) -> glam::DVec2 {
        self.localizer.position()
    }
}
impl<B: Boundary> TracksHeading for Localization<B> {
    fn heading(&self) -> Angle {
        self.localizer.heading()
    }
}
impl<B: Boundary> TracksForwardTravel for Localization<B> {
    fn forward_travel(&self) -> f64 {
        self.localizer.forward_travel()
    }
}
impl<B: Boundary> TracksVelocity for Localization<B> {
    fn linear_velocity(&self) -> f64 {
        self.localizer.linear_velocity()
    }
    fn angular_velocity(&self) -> f64 {
        self.localizer.angular_velocity()
    }
}
