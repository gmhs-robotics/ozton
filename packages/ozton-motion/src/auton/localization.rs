//! Live differential odometry and range correction shared by competition robots.
use ozton_math::{
    geometry::Pose,
    uom::si::{
        angular_velocity::radian_per_second,
        f64::{AngularVelocity, Length, Velocity},
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
    },
};
use vexide::competition::CompetitionMode;

use super::{Feedback, RangeFeedback};

/// Drive encoders, a lateral wheel, and a shared IMU, with optional range corrections.
pub struct DifferentialLocalization<L> {
    sensors: SharedSensors<(), L>,
    tracking: Tracking<'static, ozton_tracking::localizer::Filter, Distances>,
    origin: Pose,
    last: Option<Feedback>,
    last_reset: DistanceReset,
    occluded: u8,
    mechanism_occluded: u8,
    fault: Option<Fault>,
    driver_control: bool,
}
impl<L: WheelSource> DifferentialLocalization<L> {
    pub fn pause_disabled_odometry(&mut self) {
        self.tracking.localizer.invalidate_pose();
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
            origin,
            last: None,
            last_reset: DistanceReset::Idle,
            occluded: 0,
            mechanism_occluded: 0xFF,
            fault: None,
            driver_control: false,
        }
    }

    /// Poll enabled odometry. Driver control samples orientation before writing outputs.
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
        if mode == CompetitionMode::Autonomous {
            self.sensors.refresh_orientation();
        }
        self.set_driver_control(mode == CompetitionMode::Driver);
        self.mechanism_occluded = mechanism_occluded;
        self.sample(drive_positions())
    }

    pub fn refresh_orientation(&mut self) -> Option<Orientation> {
        self.sensors.refresh_orientation()
    }

    pub fn orientation(&self) -> Option<Orientation> {
        self.sensors.orientation()
    }

    pub fn imu_telemetry(&self) -> ([f64; 3], [f64; 3]) {
        self.sensors.imu_telemetry()
    }

    /// Replace the field model and rebaseline at the route's surveyed start.
    pub fn configure(
        &mut self,
        mut localizer: Localizer<'static>,
        origin: Pose,
        pose: Pose,
    ) -> bool {
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
        self.tracking.localizer.set_estimator(estimator, shadow)
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
    /// Apply a captured snapshot, including when replaying tracking off the robot.
    pub fn update(
        &mut self,
        raw: Snapshot,
        ranges: [Option<RangeObservation>; MAX_RANGES],
    ) -> Result<Option<Feedback>, Fault> {
        // This installation has no forward tracking pod. Both drive encoders and IMU are required.
        if raw.imu.is_none_or(|v| !v.is_finite())
            || raw.drive.iter().any(|v| v.is_none_or(|v| !v.is_finite()))
            || raw.lateral.is_none_or(|v| !v.is_finite())
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
        let update = match self.tracking.localizer.update(raw, &ranges) {
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
