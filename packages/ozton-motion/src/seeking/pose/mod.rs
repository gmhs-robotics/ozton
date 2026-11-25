//! Clearance-aware pose seeking for a calibrated differential drivetrain.

use std::time::Duration;

use ozton_localization::LocalizationEstimate;
use ozton_math::{
    geometry::{DVec2, Pose},
    uom::si::f64::{Angle, Length},
};

use crate::basic::differential::{DriveOptions, DriveTuning};

mod actions;
mod approach;
mod error;
mod session;
pub use actions::PathActions;
pub use error::PoseMotionError;
pub use ozton_drivetrain::{
    BrakeMode, MotionExecution, RangeResetDrive, WheelActuation, WheelCommand,
};
pub use ozton_math::path::clearance::WallContact;
use session::{PoseSession, Trajectory};

use super::Seeking;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TravelDirection {
    Forward,
    Backward,
}

impl Default for Seeking {
    fn default() -> Self {
        Self::new()
    }
}

impl Seeking {
    pub const fn new() -> Self {
        Self {
            config: crate::MotionConfig::DEFAULT,
            travel: None,
        }
    }

    /// Physical settings for this motion template.
    pub const fn config(self) -> crate::MotionConfig {
        self.config
    }

    /// Apply a robot's physical constraints, PID gains and settling configuration.
    pub const fn with_config(mut self, config: crate::MotionConfig) -> Self {
        self.config = config;
        self
    }

    #[must_use]
    pub const fn with_limits(mut self, value: crate::config::MotionLimits) -> Self {
        self.config.limits = value;
        self
    }

    #[must_use]
    pub const fn with_rates(mut self, value: crate::config::RateFilters) -> Self {
        self.config.rates = value;
        self
    }

    #[must_use]
    pub const fn with_settling(mut self, value: crate::config::SettlingConfig) -> Self {
        self.config.settling = value;
        self
    }

    #[must_use]
    pub const fn with_timing(mut self, value: crate::config::MotionTiming) -> Self {
        self.config.timing = value;
        self
    }

    #[must_use]
    pub const fn with_safety(mut self, value: crate::config::ClearanceConfig) -> Self {
        self.config.safety = value;
        self
    }

    #[must_use]
    pub const fn with_path(mut self, value: crate::config::PathConfig) -> Self {
        self.config.path = value;
        self
    }

    #[must_use]
    pub const fn with_approach(mut self, value: crate::config::ApproachConfig) -> Self {
        self.config.approach = value;
        self
    }

    #[must_use]
    pub const fn with_tuning(mut self, tuning: DriveTuning) -> Self {
        self.config.tuning = tuning;
        self
    }

    pub(super) fn options(
        self,
        drive: &impl ozton_drivetrain::model::DriveEncoders,
    ) -> Option<DriveOptions> {
        let config = self.config;
        config.valid().then(|| ())?;
        Some(config.options(drive.track_width()))
    }

    /// Direct distance/bearing PID seeking without geometric path search.
    pub async fn move_to_point<D: MotionExecution>(
        &self,
        drive: &mut D,
        point: impl Into<DVec2>,
    ) -> Result<(), PoseMotionError<D::Error>> {
        approach::drive_to_target(drive, point.into(), None, *self).await
    }

    /// Follow absolute transit points without braking at each intermediate target.
    /// The last point uses normal arrival tolerances and settling. All legs use
    /// one travel direction (forward by default). The caller must choose clear
    /// corner-cutting corridors; use a prepared path for exact curve tracking.
    pub async fn move_through_points<D: MotionExecution>(
        &self,
        drive: &mut D,
        points: &[DVec2],
        handoff_radius: Length,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        let radius = handoff_radius.value;
        if points.is_empty()
            || !radius.is_finite()
            || radius <= options.position_tolerance.value
            || points.iter().any(|point| !point.is_finite())
            || points
                .windows(2)
                .any(|pair| pair[0].distance(pair[1]) <= 2.0 * radius)
        {
            return Err(PoseMotionError::Configuration);
        }
        let reverse = self.travel == Some(TravelDirection::Backward);
        let mut session = PoseSession::new(drive, options, self.config)?;
        let result = session.through_points(points, radius, reverse).await;
        session.finish(result)
    }

    /// Approach an absolute pose using continuous Boomerang feedback.
    pub async fn move_to_pose<D: MotionExecution>(
        &self,
        drive: &mut D,
        target: Pose,
    ) -> Result<(), PoseMotionError<D::Error>> {
        if !target.is_finite() {
            return Err(PoseMotionError::Configuration);
        }
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        let field = approach::clearance(drive, self.config.safety.clearance)?;
        if field.is_some_and(|field| !field.contains_physical_pose(target)) {
            ozton_diagnostics::log_event!(Warn, "motion", "Pose target outside nominal clearance");
        }
        let reverse = self.travel.map_or_else(
            || {
                ozton_math::geometry::wrap_heading(
                    ozton_math::geometry::bearing(
                        target.position - drive.localization().pose().position,
                    ) - drive.localization().pose().heading,
                )
                .value
                .abs()
                    > core::f64::consts::FRAC_PI_2
            },
            |direction| direction == TravelDirection::Backward,
        );
        let mut session = PoseSession::new(drive, options, self.config)?;
        let result = session
            .run(
                crate::basic::differential::DifferentialTarget::DriveToPose {
                    target,
                    reverse,
                    maximum_speed: options.maximum_wheel_speed,
                },
                self.config.timing.stage_timeout,
            )
            .await;
        session.finish(result)
    }

    /// Rotate, drive straight to a position, then settle at its absolute heading.
    pub async fn move_to_pose_straight<D: MotionExecution>(
        &self,
        drive: &mut D,
        target: Pose,
    ) -> Result<(), PoseMotionError<D::Error>> {
        approach::drive_to_target(drive, target.position, Some(target.heading), *self).await
    }

    /// Follow an authored sequence of absolute field poses. Each pose settles
    /// before the next command. A failure stops the sequence, without replanning.
    pub async fn follow_poses<D: MotionExecution>(
        &self,
        drive: &mut D,
        poses: &[Pose],
    ) -> Result<(), PoseMotionError<D::Error>> {
        if poses.iter().any(|pose| !pose.is_finite()) {
            return Err(PoseMotionError::Configuration);
        }
        for (index, &pose) in poses.iter().enumerate() {
            if !drive.motion_active() {
                return Err(PoseMotionError::Cancelled);
            }
            ozton_diagnostics::log_row!(
                Info,
                "POSE_ROUTE",
                "index={index},x_m={:.5},y_m={:.5},heading_rad={:.5}",
                pose.position.x,
                pose.position.y,
                pose.heading.value
            );
            self.move_to_pose_straight(drive, pose).await?;
        }
        Ok(())
    }

    /// Drive a signed distance while preserving the starting heading.
    pub async fn drive_distance<D: MotionExecution>(
        &self,
        drive: &mut D,
        distance: Length,
    ) -> Result<(), PoseMotionError<D::Error>> {
        self.drive_distance_inner(drive, distance, None).await
    }

    /// Leave a physically verified flush contact using the ordinary
    /// distance PID and a one-shot unilateral wall constraint.
    /// Register the surveyed contact pose before this command. Virtual
    /// boundaries are never valid contact observations.
    pub async fn drive_distance_from_contact<D: MotionExecution>(
        &self,
        drive: &mut D,
        distance: Length,
        wall: WallContact,
    ) -> Result<(), PoseMotionError<D::Error>> {
        if drive.localization().wall_contact() != Some(wall) {
            return Err(PoseMotionError::Configuration);
        }
        let result = self.drive_distance_inner(drive, distance, Some(wall)).await;
        drive.localization_mut().release_wall_contact();
        result
    }

    async fn drive_distance_inner<D: MotionExecution>(
        &self,
        drive: &mut D,
        distance: Length,
        contact: Option<WallContact>,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let heading = drive.localization().pose().heading;
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        if !distance.value.is_finite() {
            return Err(PoseMotionError::Configuration);
        }
        let field = approach::clearance(drive, self.config.safety.clearance)?;
        let start = drive.localization().pose();
        let end = Pose::new(
            start.position + ozton_math::geometry::direction(heading) * distance.value,
            heading,
        );
        if let Some(wall) = contact {
            let field = field.ok_or(PoseMotionError::Configuration)?;
            if distance.value == 0.0
                || !field.contact_inward(wall, start, [0.0, distance.value, 0.0], [0.0; 3])
                || wall.inward().dot(end.position - start.position) <= 0.0
            {
                return Err(PoseMotionError::InsufficientClearance);
            }
        }
        if field.is_some_and(|f| !f.contains_translation(start, end.position)) {
            ozton_diagnostics::log_row!(
                Warn,
                "CLEARANCE_FAULT",
                "reason=planned_translation,x_m={:.5},y_m={:.5},end_x_m={:.5},end_y_m={:.5}",
                start.position.x,
                start.position.y,
                end.position.x,
                end.position.y
            );
            ozton_diagnostics::log_event!(
                Warn,
                "clearance",
                "Planned straight translation violates nominal clearance; continuing feedback"
            );
        }
        if contact.is_some() {
            drive.localization_mut().release_wall_contact();
        }
        let mut session = PoseSession::new(drive, options, self.config)?;
        let result = session
            .run(
                crate::basic::differential::DifferentialTarget::DriveDistance {
                    meters: distance,
                    heading: Some(heading),
                    maximum_speed: options.maximum_wheel_speed,
                },
                self.config.timing.stage_timeout,
            )
            .await;
        session.finish(result)
    }

    /// Execute a constant-radius forward arc with a checked swept footprint.
    pub async fn drive_arc<D: MotionExecution>(
        &self,
        drive: &mut D,
        radius: Length,
        sweep: Angle,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        if !radius.value.is_finite()
            || radius.value <= 0.0
            || !sweep.value.is_finite()
            || sweep.value.abs() > core::f64::consts::TAU
        {
            return Err(PoseMotionError::Configuration);
        }
        if let Some(field) = approach::clearance(drive, self.config.safety.clearance)? {
            let footprint = drive.footprint().ok_or(PoseMotionError::Configuration)?;
            let (min, max) = footprint
                .forward_arc_bounds(drive.localization().pose(), radius, sweep)
                .ok_or(PoseMotionError::Configuration)?;
            if !field.contains_aabb(min, max) {
                ozton_diagnostics::log_row!(
                    Warn,
                    "CLEARANCE_FAULT",
                    "reason=planned_arc,min_x_m={:.5},min_y_m={:.5},max_x_m={:.5},max_y_m={:.5},radius_m={:.5},sweep_rad={:.5}",
                    min.x,
                    min.y,
                    max.x,
                    max.y,
                    radius.value,
                    sweep.value
                );
                ozton_diagnostics::log_event!(
                    Warn,
                    "clearance",
                    "Planned arc violates nominal clearance; continuing feedback"
                );
            }
        }
        let mut session = PoseSession::new(drive, options, self.config)?;
        let result = session
            .run(
                crate::basic::differential::DifferentialTarget::DriveArc {
                    radius,
                    sweep,
                    maximum_speed: options.maximum_wheel_speed,
                },
                self.config.timing.stage_timeout,
            )
            .await;
        session.finish(result)
    }

    /// Turn to face a field point using the physical feedback controller.
    pub async fn turn_to_point<D: MotionExecution>(
        &self,
        drive: &mut D,
        target: impl Into<DVec2>,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let target = target.into();
        if !target.is_finite() {
            return Err(PoseMotionError::Configuration);
        }
        self.turn_target(
            drive,
            crate::basic::differential::DifferentialTarget::TurnToPoint { target },
        )
        .await
    }

    pub async fn turn_to_heading<D: MotionExecution>(
        &self,
        drive: &mut D,
        heading: Angle,
    ) -> Result<(), PoseMotionError<D::Error>> {
        if !heading.value.is_finite() {
            return Err(PoseMotionError::Configuration);
        }
        self.turn_target(
            drive,
            crate::basic::differential::DifferentialTarget::Turn { heading },
        )
        .await
    }

    /// Forward swing to an absolute heading with the inside wheel group held.
    /// Only drives with explicit mixed braking/velocity support can use it.
    pub async fn swing_to_heading<D: MotionExecution + WheelActuation>(
        &self,
        drive: &mut D,
        heading: Angle,
    ) -> Result<(), PoseMotionError<D::Error>>
    where
        D::Error: From<D::WheelError>,
    {
        if !heading.value.is_finite() {
            return Err(PoseMotionError::Configuration);
        }
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        let start = drive.localization().pose();
        let sweep = ozton_math::geometry::wrap_heading(heading - start.heading);
        if let Some(field) = approach::clearance(drive, self.config.safety.clearance)? {
            let footprint = drive.footprint().ok_or(PoseMotionError::Configuration)?;
            let (min, max) = footprint
                .forward_arc_bounds(start, options.track_width * 0.5, sweep)
                .ok_or(PoseMotionError::Configuration)?;
            if !field.contains_aabb(min, max) {
                ozton_diagnostics::log_event!(
                    Warn,
                    "clearance",
                    "Swing exceeds nominal clearance; continuing feedback"
                );
            }
        }
        let hold_left = sweep.value >= 0.0;
        let held = usize::from(!hold_left);
        let mut session = PoseSession::new(drive, options, self.config)?;
        let result = session
            .run_with_output(
                crate::basic::differential::DifferentialTarget::Swing { heading, hold_left },
                self.config.timing.stage_timeout,
                |drive, speeds| {
                    let mut commands = speeds.map(WheelCommand::Velocity);
                    commands[held] = WheelCommand::Brake(BrakeMode::Hold);
                    drive.drive_wheel_commands(commands).map_err(Into::into)
                },
            )
            .await;
        session.finish(result)
    }

    async fn turn_target<D: MotionExecution>(
        &self,
        drive: &mut D,
        target: crate::basic::differential::DifferentialTarget,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        let heading = match target {
            crate::basic::differential::DifferentialTarget::Turn { heading } => heading,
            crate::basic::differential::DifferentialTarget::TurnToPoint { target } => {
                ozton_math::geometry::bearing(target - drive.localization().pose().position)
            }
            _ => return Err(PoseMotionError::Configuration),
        };
        if approach::clearance(drive, self.config.safety.clearance)?
            .is_some_and(|f| !f.contains_turn(drive.localization().pose(), heading))
        {
            ozton_diagnostics::log_event!(
                Warn,
                "clearance",
                "Turn exceeds nominal clearance; continuing feedback"
            );
        }
        let mut session = PoseSession::new(drive, options, self.config)?;
        let result = session.run(target, self.config.timing.stage_timeout).await;
        session.finish(result)
    }

    pub async fn follow_path<D: MotionExecution>(
        &self,
        drive: &mut D,
        path: &ozton_math::path::Path,
        reverse: bool,
    ) -> Result<(), PoseMotionError<D::Error>> {
        self.follow_with_actions(drive, path, reverse, &mut PathActions::new())
            .await
    }

    /// Execute a continuous trajectory with nonblocking mechanism actions.
    pub async fn follow_with_actions<D: MotionExecution>(
        &self,
        drive: &mut D,
        path: &ozton_math::path::Path,
        reverse: bool,
        actions: &mut PathActions<'_>,
    ) -> Result<(), PoseMotionError<D::Error>> {
        if !actions.valid(path) {
            return Err(PoseMotionError::Configuration);
        }
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        let field = approach::clearance(drive, self.config.safety.clearance)?;
        let facing = ozton_math::qty!((if reverse { 180.0 } else { 0.0 }) deg);
        let start = path.start().pose;
        let actual = drive.localization().pose();
        if !actual.is_finite() {
            return Err(PoseMotionError::Configuration);
        }
        if actual.position.distance(start.position) > 0.08
            || ozton_math::geometry::wrap_heading(start.heading + facing - actual.heading)
                .value
                .abs()
                > 0.35
        {
            ozton_diagnostics::log_event!(
                Warn,
                "motion",
                "Trajectory start offset exceeds tuning threshold; continuing feedback"
            );
        }
        if let Some(field) = field {
            drive.brake().map_err(PoseMotionError::StopFailed)?;
            let mut slice_started = std::time::Instant::now();
            let clear = ozton_math::path::clearance::clear_path_cooperative(
                path,
                reverse,
                field,
                async || {
                    if slice_started.elapsed() >= Duration::from_millis(2) {
                        drive
                            .wait_until(std::time::Instant::now() + Duration::from_millis(1))
                            .await;
                        slice_started = std::time::Instant::now();
                    }
                    drive.motion_active()
                },
            )
            .await;
            if !drive.motion_active() {
                return Err(PoseMotionError::Cancelled);
            }
            if !clear {
                ozton_diagnostics::log_event!(
                    Warn,
                    "clearance",
                    "Planned trajectory violates clearance"
                );
                return Err(PoseMotionError::InsufficientClearance);
            }
        }
        let end = path.end().pose;
        let target = Pose::new(end.position, end.heading + facing);
        drive.localization_mut().release_wall_contact();
        let mut session = PoseSession::new(drive, options, self.config)?;
        let result = session
            .follow(
                Trajectory {
                    path,
                    reverse,
                    target,
                    timeout: self.config.timing.stage_timeout,
                },
                actions,
            )
            .await;
        session.finish(result)
    }

    /// Hold the chassis still and try to correct the pose from fresh wall ranges.
    pub async fn try_distance_reset<D: RangeResetDrive>(
        &self,
        drive: &mut D,
        timeout: Duration,
    ) -> Result<bool, PoseMotionError<D::Error>> {
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        let mut session = PoseSession::new(drive, options, self.config)?;
        let result = session.try_distance_reset(timeout).await;
        session.finish(result)
    }
}
