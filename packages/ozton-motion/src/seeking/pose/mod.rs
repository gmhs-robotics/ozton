//! Clearance-aware pose seeking for a calibrated differential drivetrain.

use std::time::{Duration, Instant};

use ozton_localization::LocalizationEstimate;
use ozton_math::{
    geometry::{DVec2, Pose},
    path::clearance::RectangularClearance,
    uom::si::f64::{Angle, Length, Velocity},
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
    #[must_use]
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

    /// Select the path tracker without changing other motion settings.
    #[must_use]
    pub const fn with_guidance(mut self, guidance: crate::PathGuidance) -> Self {
        self.config.path.guidance = guidance;
        self
    }

    #[must_use]
    pub const fn with_terminal(mut self, terminal: crate::TerminalGuidance) -> Self {
        self.config.path.terminal = terminal;
        self
    }

    /// Override only the wheel speed, preserving all other robot settings.
    #[must_use]
    pub const fn with_wheel_speed(mut self, speed: Velocity) -> Self {
        self.config.limits = self.config.limits.with_wheel_speed(speed);
        self
    }

    /// Override only the stage timeout.
    #[must_use]
    pub const fn with_stage_timeout(mut self, timeout: Duration) -> Self {
        self.config.timing = self.config.timing.with_stage_timeout(timeout);
        self
    }

    /// Construct the clearance model configured for this motion template.
    pub fn clearance<D: MotionExecution>(
        &self,
        drive: &D,
    ) -> Result<Option<RectangularClearance>, PoseMotionError<D::Error>> {
        approach::clearance(
            drive,
            self.config.safety.clearance,
            self.config.safety.movable_obstacles,
        )
    }

    /// Hold a stopped drivetrain while localization settles. Warn once on low
    /// confidence or invalid physical clearance; cancellation remains an error.
    pub async fn settle<D: MotionExecution>(
        &self,
        drive: &mut D,
        duration: Duration,
    ) -> Result<(), PoseMotionError<D::Error>> {
        self.options(drive).ok_or(PoseMotionError::Configuration)?;
        drive.brake().map_err(PoseMotionError::StopFailed)?;
        let area = self.clearance(drive)?;
        let started = Instant::now();
        let mut warned = false;
        while started.elapsed() < duration {
            if !drive.motion_active() {
                return Err(PoseMotionError::Cancelled);
            }
            let uncertain = {
                let loc = drive.localization();
                !loc.pose_trusted()
                    || area.is_some_and(|area| !area.contains_physical_pose(loc.pose()))
            };
            if !warned && uncertain {
                warned = true;
                ozton_diagnostics::log_event!(
                    Warn,
                    "localization",
                    "Initial pose confidence or clearance is low"
                );
            }
            drive
                .wait_until(Instant::now() + self.config.timing.period)
                .await;
        }
        Ok(())
    }

    /// Check an in-place turn, straight translation, and final turn before
    /// executing the ordinary straight pose controller.
    pub async fn move_to_pose_straight_checked<D: MotionExecution>(
        &self,
        drive: &mut D,
        target: Pose,
    ) -> Result<(), PoseMotionError<D::Error>> {
        use ozton_math::geometry::{bearing, wrap_heading};

        let start = drive.localization().pose();
        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        if !start.is_finite() || !target.is_finite() {
            return Err(PoseMotionError::Configuration);
        }
        if let Some(area) = self.clearance(drive)? {
            let delta = target.position - start.position;
            if delta.length() <= options.position_tolerance.value {
                if !area.contains_turn(start, target.heading) {
                    return Err(PoseMotionError::InsufficientClearance);
                }
            } else {
                let bearing = bearing(delta);
                let reverse = self.travel.map_or_else(
                    || {
                        wrap_heading(bearing - start.heading).value.abs()
                            > core::f64::consts::FRAC_PI_2
                    },
                    |direction| direction == TravelDirection::Backward,
                );
                let heading = wrap_heading(
                    bearing + ozton_math::qty!((if reverse { 180.0 } else { 0.0 }) deg),
                );
                if !area.contains_turn(start, heading)
                    || !area
                        .contains_translation(Pose::new(start.position, heading), target.position)
                    || !area.contains_turn(Pose::new(target.position, heading), target.heading)
                {
                    return Err(PoseMotionError::InsufficientClearance);
                }
            }
        }
        self.move_to_pose_straight(drive, target).await
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
        let result = session.through_points(points, radius, reverse, false).await;
        session.finish(result)
    }

    /// Chain straight legs, carrying speed through collinear transit points.
    /// At a direction change, settle translation and acquire the next bearing
    /// before accelerating. The caller supplies only positions, not turn angles.
    /// Reject nominally obstructed translations and pivot turns before driving.
    pub async fn move_through_points_straight<D: MotionExecution>(
        &self,
        drive: &mut D,
        points: &[DVec2],
        handoff_radius: Length,
    ) -> Result<(), PoseMotionError<D::Error>> {
        use ozton_math::geometry::{bearing, wrap_heading};

        let options = self.options(drive).ok_or(PoseMotionError::Configuration)?;
        let radius = handoff_radius.value;
        let start = drive.localization().pose();
        if points.is_empty()
            || !start.is_finite()
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
        let facing = ozton_math::qty!((if reverse { 180.0 } else { 0.0 }) deg);
        let tolerance = options.heading_tolerance.value;
        let field = self.clearance(drive)?;

        let mut nominal = start;
        for &point in points {
            if nominal.position.distance(point) <= options.position_tolerance.value {
                continue;
            }
            let heading = wrap_heading(bearing(point - nominal.position) + facing);
            let aligned = Pose::new(nominal.position, heading);
            if field.as_ref().is_some_and(|area| {
                !area.contains_turn(nominal, heading) || !area.contains_translation(aligned, point)
            }) {
                return Err(PoseMotionError::InsufficientClearance);
            }
            nominal = Pose::new(point, heading);
        }

        let mut session = PoseSession::new(drive, options, self.config)?;
        let result = async {
            let mut index = 0;
            while index < points.len() {
                let pose = session.drive.localization().pose();
                if pose.position.distance(points[index]) <= options.position_tolerance.value {
                    index += 1;
                    continue;
                }
                let origin = if index == 0 {
                    start.position
                } else {
                    points[index - 1]
                };
                let end = straight_run_end(origin, &points[index..], tolerance) + index;
                let heading = wrap_heading(bearing(points[index] - pose.position) + facing);
                let aligned = Pose::new(pose.position, heading);
                if field.as_ref().is_some_and(|area| {
                    !area.contains_turn(pose, heading)
                        || !area.contains_translation(aligned, points[end])
                }) {
                    return Err(PoseMotionError::InsufficientClearance);
                }
                if wrap_heading(heading - pose.heading).value.abs() > tolerance {
                    session
                        .run(
                            crate::basic::differential::DifferentialTarget::Turn { heading },
                            self.config.timing.stage_timeout,
                        )
                        .await?;
                }
                ozton_diagnostics::log_row!(Info, "STRAIGHT_CHAIN", "first={index},last={end}");
                session
                    .through_points(&points[index..=end], radius, reverse, true)
                    .await?;
                index = end + 1;
            }
            Ok(())
        }
        .await;
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
        let field = approach::clearance(drive, self.config.safety.clearance, false)?;
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
        let field = approach::clearance(drive, self.config.safety.clearance, false)?;
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
        if let Some(field) = approach::clearance(drive, self.config.safety.clearance, false)? {
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
        if let Some(field) = approach::clearance(drive, self.config.safety.clearance, false)? {
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
        if approach::clearance(drive, self.config.safety.clearance, false)?
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

    /// Load a host-generated path after checking that its limits match the robot.
    pub fn prepare_path<D: MotionExecution>(
        &self,
        drive: &D,
        samples: &'static [ozton_math::path::PathSample],
        generated_limits: [f64; 7],
    ) -> Result<ozton_math::path::Path, PoseMotionError<D::Error>> {
        let limits = self.config.path_limits(drive.track_width());
        if !limits.matches(generated_limits) {
            return Err(PoseMotionError::Configuration);
        }
        ozton_math::path::Path::from_precomputed(samples, limits)
            .map_err(|_| PoseMotionError::Configuration)
    }

    /// Follow a prepared path using the direction selected by the builder.
    pub async fn follow<D: MotionExecution>(
        &self,
        drive: &mut D,
        path: &ozton_math::path::Path,
    ) -> Result<(), PoseMotionError<D::Error>> {
        self.follow_path(drive, path, self.travel == Some(TravelDirection::Backward))
            .await
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
        let field = approach::clearance(drive, self.config.safety.clearance, false)?;
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
        if let Some(mut field) = field {
            if self.config.safety.movable_obstacles {
                for polygon in drive
                    .localization()
                    .movable_obstacles()
                    .into_iter()
                    .flatten()
                {
                    field = field
                        .with_polygon(polygon)
                        .ok_or(PoseMotionError::Configuration)?;
                }
            }
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
        let before = session.drive.localization().pose();
        let result = session.try_distance_reset(timeout).await;
        let after = session.drive.localization().pose();
        if let Ok(accepted) = &result {
            ozton_diagnostics::log_row!(
                Info,
                "DISTANCE_RESET_RESULT",
                "accepted={accepted},before_x={:.6},before_y={:.6},after_x={:.6},after_y={:.6},delta_m={:.6}",
                before.position.x,
                before.position.y,
                after.position.x,
                after.position.y,
                before.position.distance(after.position)
            );
        }
        session.finish(result)
    }
}

/// The last consecutive point on the current straight leg.
fn straight_run_end(start: DVec2, points: &[DVec2], tolerance: f64) -> usize {
    use ozton_math::geometry::{bearing, wrap_heading};

    let heading = bearing(points[0] - start);
    let mut end = 0;
    for (index, pair) in points.windows(2).enumerate() {
        let next = bearing(pair[1] - pair[0]);
        if wrap_heading(next - heading).value.abs() > tolerance {
            break;
        }
        end = index + 1;
    }
    end
}

#[cfg(test)]
mod straight_chain_tests {
    use super::*;

    #[test]
    fn collinear_transit_points_chain_but_right_angles_and_reversals_stop() {
        let points = [
            DVec2::new(0.0, 1.0),
            DVec2::new(0.0, 2.0),
            DVec2::new(-1.0, 2.0),
            DVec2::new(-2.0, 2.0),
            DVec2::new(-1.0, 2.0),
        ];
        assert_eq!(straight_run_end(DVec2::ZERO, &points, 0.03), 1);
        assert_eq!(straight_run_end(points[1], &points[2..], 0.03), 1);
        assert_eq!(straight_run_end(points[3], &points[4..], 0.03), 0);
    }
}
