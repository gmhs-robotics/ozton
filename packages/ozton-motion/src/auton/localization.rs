//! Live differential odometry and range correction shared by competition robots.
use ozton_drivetrain::model::DriveEncoders;
use ozton_math::{
    geometry::Pose,
    uom::si::{
        angle::radian,
        angular_velocity::radian_per_second,
        f64::{Angle, AngularVelocity, Length, Velocity},
        length::meter,
        velocity::meter_per_second,
    },
};
use ozton_tracking::{
    TracksForwardTravel,
    localizer::{
        DistanceReset, Estimator, Fault, Localizer, MAX_RANGES, RangeObservation,
        RangeSensorConfig, Snapshot, Tracking,
        vex::{Distances, Orientation, SharedSensors, WheelSource},
        wheels::WheelOdometry,
    },
};
use vexide::competition::CompetitionMode;

use super::{Feedback, RangeFeedback, tracking_builder::RangeUses};

struct Layout {
    wheels: WheelOdometry,
    range_uses: RangeUses,
    orientation: bool,
    orientation_pending: bool,
}

/// Differential robot tracking with independently owned sensors and optional range correction.
/// Use [`super::TrackingBuilder`] to assemble a wheel and sensor layout.
pub struct DifferentialLocalization<L> {
    sensors: SharedSensors<(), L>,
    tracking: Tracking<'static, ozton_tracking::localizer::Filter, Distances>,
    layout: Option<Layout>,
    origin: Pose,
    imu_heading_offset: Angle,
    last: Option<Feedback>,
    last_reset: DistanceReset,
    occluded: u8,
    mechanism_occluded: u8,
    fault: Option<Fault>,
    driver_control: bool,
}

impl<L> ozton_tracking::Tracking for DifferentialLocalization<L> {}

impl<L: WheelSource> ozton_drivetrain::DriverTracking for DifferentialLocalization<L> {
    fn driver_orientation(&mut self) -> (Option<Angle>, Option<Angle>) {
        let sample = self.refresh_orientation();
        (
            sample.and_then(|sample| sample.heading),
            sample.and_then(|sample| sample.roll),
        )
    }
}
impl<L: WheelSource> DifferentialLocalization<L> {
    pub fn pause_disabled_odometry(&mut self) {
        self.tracking.localizer.invalidate_pose();
        if let Some(layout) = &mut self.layout {
            layout.orientation_pending = false;
        }
        self.last = None;
    }

    pub fn new(
        mut sensors: SharedSensors<(), L>,
        distances: Distances,
        localizer: Localizer<'static>,
    ) -> Self {
        let origin = Pose::default();
        sensors.set_field_origin(origin);
        Self {
            sensors,
            tracking: Tracking::new(localizer).with_distances(distances),
            layout: None,
            origin,
            imu_heading_offset: Angle::new::<radian>(0.0),
            last: None,
            last_reset: DistanceReset::Idle,
            occluded: 0,
            mechanism_occluded: 0xFF,
            fault: None,
            driver_control: false,
        }
    }

    pub(super) fn with_layout(
        mut self,
        wheels: WheelOdometry,
        range_uses: RangeUses,
        orientation: bool,
    ) -> Self {
        range_uses.apply(&mut self.tracking.localizer);
        self.layout = Some(Layout {
            wheels,
            range_uses,
            orientation,
            orientation_pending: false,
        });
        self
    }

    /// Borrow the drivetrain only for this tick and sample the configured drive encoders once.
    pub fn poll_with_drive(
        &mut self,
        mode: CompetitionMode,
        mechanism_occluded: u8,
        drive: &mut impl DriveEncoders,
    ) -> Result<Option<Feedback>, Fault> {
        self.poll(mode, mechanism_occluded, || drive.tracking_positions())
    }

    /// Poll a layout that uses separate tracking wheels. Drive travel is unavailable.
    pub fn poll_wheels(
        &mut self,
        mode: CompetitionMode,
        mechanism_occluded: u8,
    ) -> Result<Option<Feedback>, Fault> {
        self.poll(mode, mechanism_occluded, || [None; 2])
    }

    /// Poll enabled odometry. A builder-configured tracker uses an orientation
    /// sample refreshed before driver output, or refreshes it here if needed.
    /// Range corrections run only outside driver mode.
    pub fn poll(
        &mut self,
        mode: CompetitionMode,
        mechanism_occluded: u8,
        drive_positions: impl FnOnce() -> [Option<Length>; 2],
    ) -> Result<Option<Feedback>, Fault> {
        if mode == CompetitionMode::Disabled {
            return Ok(None);
        }
        let needs_orientation = self
            .layout
            .as_ref()
            .map_or(mode == CompetitionMode::Autonomous, |layout| {
                !layout.orientation_pending
            });
        if needs_orientation {
            self.refresh_orientation();
        }
        if let Some(layout) = &mut self.layout {
            layout.orientation_pending = false;
        }
        self.set_driver_control(mode == CompetitionMode::Driver);
        self.mechanism_occluded = mechanism_occluded;
        let drive = if self.uses_drive_encoders() {
            drive_positions()
        } else {
            [None; 2]
        };
        self.sample(drive)
    }

    pub(super) fn uses_drive_encoders(&self) -> bool {
        self.layout
            .as_ref()
            .is_none_or(|layout| layout.wheels.uses_drive())
    }

    pub fn refresh_orientation(&mut self) -> Option<Orientation> {
        if let Some(layout) = &mut self.layout {
            layout.orientation_pending = true;
            if !layout.orientation {
                return layout
                    .wheels
                    .uses_imu_heading()
                    .then(|| self.sensors.refresh_heading())
                    .flatten();
            }
        }
        self.sensors.refresh_orientation()
    }

    pub fn orientation(&self) -> Option<Orientation> {
        self.sensors.orientation()
    }

    /// Align the existing IMU reading with a known field heading without recalibrating.
    pub(super) fn register_start(&mut self, pose: Pose) -> Result<(), super::DriveMotionFault> {
        let heading = self
            .sensors
            .refresh_heading()
            .and_then(|sample| sample.heading)
            .filter(|heading| heading.value.is_finite())
            .ok_or(super::DriveMotionFault::MissingHeading)?;
        if !self.set_pose(pose) {
            return Err(super::DriveMotionFault::InvalidCommand);
        }
        self.imu_heading_offset =
            Angle::new::<radian>(pose.heading) - ozton_math::geometry::compass_heading(heading);
        Ok(())
    }

    pub(super) fn imu_field_heading(&mut self) -> Option<Angle> {
        let heading = self.sensors.refresh_heading()?.heading?;
        heading
            .value
            .is_finite()
            .then(|| ozton_math::geometry::compass_heading(heading) + self.imu_heading_offset)
    }

    /// Read IMU rates and acceleration when the active diagnostics sink requests them.
    pub fn imu_telemetry(&self) -> ([f64; 3], [f64; 3]) {
        self.sensors.imu_telemetry()
    }

    /// Use the cached control sample when it includes tilt, otherwise read a
    /// full orientation for the active diagnostics sink.
    pub fn diagnostic_orientation(&self) -> Option<Orientation> {
        self.sensors.diagnostic_orientation()
    }

    /// Replace the field model and rebaseline at the route's surveyed start.
    pub fn configure(
        &mut self,
        mut localizer: Localizer<'static>,
        origin: Pose,
        pose: Pose,
    ) -> bool {
        if let Some(layout) = &self.layout {
            if localizer.range_count() != self.tracking.distances.len() {
                return false;
            }
            layout.range_uses.apply(&mut localizer);
        }
        if !origin.is_finite() || !localizer.set_pose(pose) {
            return false;
        }
        self.sensors.set_field_origin(origin);
        self.tracking.localizer = localizer;
        self.origin = origin;
        self.last = None;
        self.last_reset = DistanceReset::Idle;
        true
    }
    pub fn latest_range(&self, index: usize) -> Option<RangeFeedback> {
        let raw = self.tracking.distances.latest(index)?;
        let feedback = self.last?.ranges.get(index).copied().flatten()?;
        (!raw.occluded && raw.timestamp == feedback.timestamp).then_some(feedback)
    }

    /// Number of configured range indices, in wiring order.
    pub fn range_count(&self) -> usize {
        self.tracking.localizer.range_count()
    }

    pub fn range_config(&self, index: usize) -> Option<RangeSensorConfig> {
        self.tracking.localizer.range_config(index)
    }
    pub fn cancel_distance_reset(&mut self) {
        self.tracking.localizer.cancel_distance_reset();
    }
    pub fn origin(&self) -> Pose {
        self.origin
    }
    pub fn pose_trusted(&self) -> bool {
        self.tracking.localizer.pose_trust() == ozton_tracking::localizer::PoseTrust::Trusted
    }
    pub fn set_estimator(&mut self, estimator: Estimator, shadow: bool) -> bool {
        if !self.tracking.localizer.set_estimator(estimator, shadow) {
            return false;
        }
        if let Some(layout) = &self.layout {
            layout.range_uses.apply(&mut self.tracking.localizer);
        }
        true
    }
    pub fn pose(&self) -> Pose {
        self.tracking.localizer.pose()
    }
    pub fn set_pose(&mut self, pose: Pose) -> bool {
        if !self.tracking.localizer.set_pose(pose) {
            return false;
        }
        self.last = None;
        self.last_reset = DistanceReset::Idle;
        true
    }
    pub fn set_occluded(&mut self, mask: u8) {
        self.occluded = mask;
    }
    pub fn request_distance_reset(&mut self) {
        self.tracking.localizer.request_distance_reset();
        self.last_reset = DistanceReset::Idle;
    }
    pub fn request_distance_reset_axes(&mut self, minimum_axes: u8) -> bool {
        if !self
            .tracking
            .localizer
            .request_distance_reset_axes(minimum_axes)
        {
            return false;
        }
        self.last_reset = DistanceReset::Idle;
        true
    }
    pub fn distance_reset_axes(&self) -> u8 {
        match self.last_reset {
            DistanceReset::Applied { axes } => axes,
            _ => 0,
        }
    }
    pub fn fault(&self) -> Option<Fault> {
        self.fault
    }
    pub fn needs_relocalization(&self) -> bool {
        self.tracking.localizer.pose_trust()
            == ozton_tracking::localizer::PoseTrust::NeedsRelocalization
    }
    // Keep the localizer out of the competition future while specializing wheel reads here.
    #[inline(never)]
    fn sample(&mut self, drive: [Option<Length>; 2]) -> Result<Option<Feedback>, Fault> {
        self.tracking
            .localizer
            .set_corrections_enabled(!self.driver_control);
        let (raw, ranges) = if self.driver_control {
            (self.sensors.capture_odometry(drive), [None; MAX_RANGES])
        } else {
            self.sensors.capture_with_distances(
                &mut self.tracking.distances,
                drive,
                self.occluded | self.mechanism_occluded,
            )
        };
        let result = self.update(raw, ranges);
        match &result {
            Err(fault) if self.fault != Some(*fault) => {
                self.fault = Some(*fault);
                ozton_runtime::diagnostic_event!("tracking_fault={fault:?},pose_trusted=0");
            }
            Ok(Some(_)) if self.fault.take().is_some() => {
                ozton_runtime::diagnostic_event!(
                    "tracking_recovered,needs_relocalization={}",
                    u8::from(self.needs_relocalization())
                );
            }
            _ => {}
        }
        result
    }

    /// Keep encoder/IMU odometry during driver control; external corrections wait.
    pub fn set_driver_control(&mut self, enabled: bool) {
        self.driver_control = enabled;
        self.tracking.localizer.set_corrections_enabled(!enabled);
    }
    /// Apply a captured snapshot and read any mounted pods from the builder.
    /// Legacy axis-based trackers can replay these snapshots directly.
    pub fn update(
        &mut self,
        raw: Snapshot,
        ranges: [Option<RangeObservation>; MAX_RANGES],
    ) -> Result<Option<Feedback>, Fault> {
        // Preserve the original fixed-input contract for callers of new().
        if self.layout.is_none()
            && (raw.imu.is_none_or(|v| !v.is_finite())
                || raw.drive.iter().any(|v| v.is_none_or(|v| !v.is_finite()))
                || raw.lateral.is_none_or(|v| !v.is_finite()))
        {
            self.tracking.localizer.invalidate_pose();
            self.last = None;
            return Err(if raw.imu.is_none_or(|v| !v.is_finite()) {
                Fault::Heading
            } else if raw.lateral.is_none_or(|v| !v.is_finite()) {
                Fault::Lateral
            } else {
                Fault::Forward
            });
        }
        let result = if let Some(layout) = &mut self.layout {
            layout.wheels.capture();
            self.tracking.localizer.update_with_odometry(
                raw,
                &ranges,
                |before, after, config, dt| layout.wheels.delta(before, after, config, dt),
            )
        } else {
            self.tracking.localizer.update(raw, &ranges)
        };
        let update = match result {
            Ok(update) => update,
            Err(Fault::Initializing) => return Ok(None),
            Err(fault) => {
                self.last = None;
                return Err(fault);
            }
        };
        if update.reset != DistanceReset::Idle {
            self.last_reset = update.reset;
        }
        let mut feedback_ranges = [None; MAX_RANGES];
        for (index, sample) in ranges.into_iter().enumerate() {
            if let Some(sample) = sample
                && let Some(config) = self.tracking.localizer.range_config(index)
                && let Ok(distance) = config.relative_distance(sample, raw.time)
            {
                feedback_ranges[index] = Some(RangeFeedback {
                    distance,
                    timestamp: sample.timestamp,
                });
            }
        }
        let feedback = Feedback {
            time: raw.time,
            pose: update.pose,
            forward_travel: Length::new::<meter>(self.tracking.localizer.forward_travel()),
            forward_velocity: Velocity::new::<meter_per_second>(update.forward_velocity),
            lateral_velocity: Velocity::new::<meter_per_second>(update.lateral_velocity),
            angular_velocity: AngularVelocity::new::<radian_per_second>(update.angular_velocity),
            ranges: feedback_ranges,
            reset_accepted: update.distance_reset,
        };
        self.last = Some(feedback);
        Ok(Some(feedback))
    }
    pub fn cached_feedback(&self) -> (Option<Feedback>, Option<bool>) {
        let reset = match self.last_reset {
            DistanceReset::Applied { .. } => Some(true),
            DistanceReset::Rejected => Some(false),
            _ => None,
        };
        (self.last, reset)
    }

    pub fn raw_range(&self, index: usize) -> Option<RangeObservation> {
        self.tracking.distances.latest(index)
    }
}
