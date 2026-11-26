//! One timestamped localization pipeline for a robot's motion and diagnostics.

use std::{borrow::Cow, time::Duration};

use glam::DVec2;
use ozton_math::{
    geometry::{BodyDelta, Pose, wrap_heading},
    localization::{
        observation::{TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity},
        range::Surface,
    },
    path::clearance::{WallContact, WallContactConstraint},
    uom::si::f64::{Angle, Length, Time},
};

mod estimate;
mod history;
mod ranges;
mod reset;
mod velocity;
pub use estimate::{Ekf, EkfConfig, Estimator, Filter, Mcl, Observations, Odometry, PoseEstimator};
use history::PoseHistory;
pub use ozton_math::localization::observation::{
    MAX_RANGES, RangeObservation, RangeRoles, RangeSensorConfig,
};
pub use ranges::RangeReport;
use ranges::{RangeMasks, RangeTick, inspect};
pub use reset::DsrConfig;
use reset::{ResetResult, ResetTick, ResetWindow};
use velocity::VelocityEstimator;
pub use velocity::{MotionObservations, VelocityConfig};

/// Cumulative drive positions and heading at a localization sample time.
#[derive(Debug, Clone, Copy, Default)]
pub struct Snapshot {
    pub time: Duration,
    pub drive: [Option<Length>; 2],
    pub imu: Option<Angle>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    Initializing,
    Time,
    InvalidEstimate,
}

/// Sensor recovery does not prove that the robot stayed at its old field position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PoseTrust {
    Unregistered,
    Trusted,
    NeedsRelocalization,
}

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub maximum_gap: Duration,
    pub estimator: Estimator,
    /// Run the estimator for diagnostics while motion uses dead reckoning.
    pub shadow: bool,
}

impl Config {
    /// Select odometry and an external pose estimator.
    pub const fn new(maximum_gap: Duration, estimator: Estimator, shadow: bool) -> Self {
        Self {
            maximum_gap,
            estimator,
            shadow,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Correction {
    None,
    Accepted,
    Rejected,
    Shadow,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DistanceReset {
    Idle,
    Waiting,
    Applied { axes: u8 },
    Rejected,
}

#[derive(Debug, Clone, Copy)]
pub struct Update {
    pub raw: Snapshot,
    pub odometry: Pose,
    pub predicted: Pose,
    pub pose: Pose,
    pub candidate: Option<Pose>,
    pub correction: Correction,
    pub distance_reset: bool,
    /// Independent observed translation constraints: 0, 1 or 2. Heading is never invented.
    pub distance_reset_axes: u8,
    pub reset: DistanceReset,
    pub forward_velocity: f64,
    pub lateral_velocity: f64,
    pub angular_velocity: f64,
    pub ranges: [Option<RangeReport>; MAX_RANGES],
}

/// Composes independent sensors without observing the same packet twice.
pub struct Localizer<'a, E = Filter> {
    config: Config,
    map: Cow<'a, [Surface]>,
    sensors: Cow<'a, [RangeSensorConfig]>,
    filter: E,
    pose: Pose,
    odometry: Pose,
    filter_pose: Pose,
    previous: Option<Snapshot>,
    history: PoseHistory,
    range_times: [Option<Duration>; MAX_RANGES],
    forward_travel: f64,
    velocity: VelocityEstimator,
    forward_velocity: f64,
    angular_velocity: f64,
    reset: Option<ResetWindow>,
    dsr_config: DsrConfig,

    trust: PoseTrust,
    corrections_enabled: bool,
    contact: Option<WallContactConstraint>,
    range_masks: RangeMasks,
    filter_ranges: u8,
}

impl<'a> Localizer<'a> {
    pub fn new(
        config: Config,
        pose: Pose,
        map: &'a [Surface],
        sensors: &'a [RangeSensorConfig],
    ) -> Option<Self> {
        Self::from_parts(config, pose, Cow::Borrowed(map), Cow::Borrowed(sensors))
    }

    /// Builds a localizer that owns its configured geometry and sensor mounts.
    pub fn new_owned(
        config: Config,
        pose: Pose,
        map: Vec<Surface>,
        sensors: Vec<RangeSensorConfig>,
    ) -> Option<Localizer<'static>> {
        Localizer::from_parts(config, pose, Cow::Owned(map), Cow::Owned(sensors))
    }

    fn from_parts(
        config: Config,
        pose: Pose,
        map: Cow<'a, [Surface]>,
        mut sensors: Cow<'a, [RangeSensorConfig]>,
    ) -> Option<Self> {
        if config.maximum_gap.is_zero()
            || !pose.is_finite()
            || map.len() > u128::BITS as usize
            || sensors.len() > MAX_RANGES
            || map.iter().any(|surface| !surface.is_valid())
            || sensors.iter().any(|sensor| !sensor.valid(&map))
        {
            return None;
        }
        // Resolve height eligibility once, including direct Localizer callers.
        for sensor in sensors.to_mut() {
            *sensor = sensor.with_visible_surfaces(&map);
        }
        let range_masks = RangeMasks::new(&sensors);
        let pose = Pose {
            heading: wrap_heading(pose.heading),
            ..pose
        };
        let filter = Filter::new(pose, config.estimator)?;
        let filter_ranges = range_masks.for_roles(filter.range_roles());
        Some(Self {
            config,
            map,
            sensors,
            filter,
            pose,
            odometry: pose,
            filter_pose: pose,
            previous: None,
            history: PoseHistory::default(),
            range_times: [None; MAX_RANGES],
            forward_travel: 0.0,
            velocity: VelocityEstimator::default(),
            forward_velocity: 0.0,
            angular_velocity: 0.0,
            reset: None,
            dsr_config: DsrConfig::default(),

            trust: PoseTrust::Unregistered,
            corrections_enabled: true,
            contact: None,
            range_masks,
            filter_ranges,
        })
    }
}

impl<'a, E: PoseEstimator> Localizer<'a, E> {
    pub fn with_filter<N: PoseEstimator>(self, mut filter: N) -> Localizer<'a, N> {
        filter.reset(self.pose);
        filter.set_contact_constraint(self.contact);
        let filter_ranges = self.range_masks.for_roles(filter.range_roles());
        let Self {
            config,
            map,
            sensors,
            pose,
            odometry,
            previous,
            history,
            range_times,
            forward_travel,
            velocity,
            forward_velocity,
            angular_velocity,
            dsr_config,

            trust,
            corrections_enabled,
            contact,
            range_masks,
            ..
        } = self;
        Localizer {
            config,
            map,
            sensors,
            filter,
            pose,
            odometry,
            filter_pose: pose,
            previous,
            history,
            range_times,
            forward_travel,
            velocity,
            forward_velocity,
            angular_velocity,
            reset: None,
            dsr_config,

            trust,
            corrections_enabled,
            contact,
            range_masks,
            filter_ranges,
        }
    }

    /// Change DSR collection requirements and discard any in-flight collection.
    pub fn set_dsr_config(&mut self, config: DsrConfig) -> bool {
        if !config.valid() {
            return false;
        }
        self.dsr_config = config;
        self.reset = None;
        true
    }

    pub fn dsr_config(&self) -> DsrConfig {
        self.dsr_config
    }

    /// Whether an eligible mapped sensor is available for these roles.
    pub(crate) fn has_mapped_ranges(&self, roles: RangeRoles) -> bool {
        self.range_masks.for_roles(roles) != 0
    }

    /// Skip automatic range inspection during differential dead reckoning.
    /// Explicit resets still use their own independently compiled role.
    pub(crate) fn disable_range_inspection(&mut self) {
        self.filter_ranges = 0;
    }

    /// Covariance of the published position, if the active estimator provides it.
    pub fn position_covariance(&self) -> Option<glam::DMat2> {
        if self.config.shadow || !self.corrections_enabled {
            return None;
        }
        self.filter.position_covariance()
    }

    /// Model standard deviations in chassis-right, forward and yaw order (SI).
    pub fn velocity_uncertainty(&self) -> [f64; 3] {
        self.velocity.standard_deviation()
    }

    /// Configure the shared velocity estimator and reset its covariance.
    pub fn set_velocity_config(&mut self, config: VelocityConfig) -> bool {
        self.velocity.set_config(config)
    }

    /// Initial or currently selected pose estimator.
    pub fn estimator(&self) -> Estimator {
        self.config.estimator
    }

    pub fn pose_trust(&self) -> PoseTrust {
        self.trust
    }

    /// Keep live odometry while suspending external observations and filter work.
    /// Driver control can use this to bound tracking cost. Re-enabling corrections
    /// re-seeds the filter at the current pose without restoring lost pose trust.
    pub fn set_corrections_enabled(&mut self, enabled: bool) {
        if self.corrections_enabled != enabled {
            self.reset = None;
            self.filter_pose = self.pose;
            if enabled {
                self.filter.reset(self.pose);
                self.filter.set_contact_constraint(self.contact);
            }
            self.corrections_enabled = enabled;
        }
    }

    /// Explicitly register a surveyed pose with set_pose after an outage.
    pub fn invalidate_pose(&mut self) {
        self.trust = PoseTrust::NeedsRelocalization;
        self.release_wall_contact();
        self.previous = None;
        self.history.clear();
        self.reset = None;
        self.velocity.reset();
        self.forward_velocity = 0.0;
        self.angular_velocity = 0.0;
    }

    pub fn pose(&self) -> Pose {
        self.pose
    }

    /// Whether range corrections are currently enabled by the tracking policy.
    pub fn corrections_enabled(&self) -> bool {
        self.corrections_enabled
    }

    pub fn estimator_name(&self) -> &'static str {
        self.filter.name(self.config.shadow)
    }

    pub fn range_max_age(&self, index: usize) -> Option<Time> {
        self.sensors.get(index).map(|sensor| sensor.gate.max_age)
    }

    pub fn range_count(&self) -> usize {
        self.sensors.len()
    }

    pub fn range_config(&self, index: usize) -> Option<RangeSensorConfig> {
        self.sensors.get(index).copied()
    }

    /// Applies a known pose, e.g. after verified contact, and rebaselines odometry.
    pub fn set_pose(&mut self, pose: Pose) -> bool {
        if !pose.is_finite() {
            return false;
        }
        let pose = Pose {
            heading: wrap_heading(pose.heading),
            ..pose
        };
        self.trust = PoseTrust::Trusted;
        self.contact = None;
        self.pose = pose;
        self.odometry = pose;
        self.filter_pose = pose;
        self.filter.reset(pose);
        self.previous = None;
        self.history.clear();
        self.velocity.reset();
        self.forward_velocity = 0.0;
        self.angular_velocity = 0.0;
        self.reset = None;
        true
    }

    /// Register a verified flush-contact pose and condition the estimator's
    /// initial uncertainty. This is a physical observation, not a range-map
    /// exclusion or a permanent reduction of process noise.
    pub fn set_pose_at_contact(&mut self, pose: Pose, contact: WallContactConstraint) -> bool {
        if !self.set_pose(pose) {
            return false;
        }
        self.contact = Some(contact);
        self.filter.reset_at_contact(self.pose, contact);
        true
    }

    pub fn wall_contact(&self) -> Option<WallContact> {
        self.contact.map(WallContactConstraint::wall)
    }

    pub fn release_wall_contact(&mut self) {
        self.contact = None;
        self.filter.set_contact_constraint(None);
    }

    /// Attempt an immediate geometric DSR on the next fresh sample.
    pub fn request_distance_reset(&mut self) {
        if self.corrections_enabled && self.range_masks.reset() != 0 {
            self.reset.get_or_insert_with(ResetWindow::default);
        }
    }

    /// Request one or two independently observed translation axes. Requiring two
    /// waits for both, instead of publishing an earlier one-axis partial reset.
    /// A repeated ordinary reset request does not weaken this requirement.
    /// Rejected collections retry at rest until accepted or explicitly canceled;
    /// the caller must enforce a timeout, as the autonomous runner does.
    pub fn request_distance_reset_axes(&mut self, minimum_axes: u8) -> bool {
        if !self.corrections_enabled
            || self.range_masks.reset() == 0
            || !(1..=2).contains(&minimum_axes)
        {
            return false;
        }
        self.reset
            .get_or_insert_with(|| ResetWindow::new(minimum_axes))
            .require_axes(minimum_axes);
        true
    }

    /// Cancel a pending reset, e.g. after an optional DSR times out.
    pub fn cancel_distance_reset(&mut self) {
        self.reset = None;
    }

    /// Update from cumulative observations, with one caller-supplied body-motion model.
    /// The model runs only after two valid, consecutive timestamps are available.
    pub fn update(
        &mut self,
        raw: Snapshot,
        observations: &[Option<RangeObservation>],
        rates: MotionObservations,
        motion: impl FnOnce(Snapshot, Snapshot) -> BodyDelta,
    ) -> Result<Update, Fault> {
        let result = self.update_inner(raw, observations, rates, motion);
        if result
            .as_ref()
            .is_err_and(|fault| !matches!(fault, Fault::Initializing))
        {
            self.invalidate_pose();
        }
        result
    }

    fn update_inner(
        &mut self,
        raw: Snapshot,
        observations: &[Option<RangeObservation>],
        rates: MotionObservations,
        motion: impl FnOnce(Snapshot, Snapshot) -> BodyDelta,
    ) -> Result<Update, Fault> {
        let Some(previous) = self.previous else {
            self.previous = Some(raw);
            self.history.record(raw.time, self.odometry);
            return Err(Fault::Initializing);
        };
        let dt = raw
            .time
            .checked_sub(previous.time)
            .filter(|dt| !dt.is_zero())
            .ok_or(Fault::Time)?;
        self.previous = Some(raw);
        if dt > self.config.maximum_gap {
            self.forward_velocity = 0.0;
            self.angular_velocity = 0.0;
            return Err(Fault::Time);
        }
        let movement = motion(previous, raw);
        let seconds = dt.as_secs_f64();
        let odometry = self.odometry.integrate(movement);
        let predicted = if self.pose == self.odometry {
            odometry
        } else {
            self.pose.integrate(movement)
        };
        let predicted = self
            .contact
            .map_or(predicted, |contact| contact.project(predicted));
        // Odometry and an uncorrected filter share the same pose in the common
        // path. Reuse integration rather than evaluating the same trig again.
        let filter_prediction = if !self.corrections_enabled || self.filter_pose == self.pose {
            predicted
        } else {
            self.filter_pose.integrate(movement)
        };
        let filter_prediction = self.contact.map_or(filter_prediction, |contact| {
            contact.project(filter_prediction)
        });
        if !odometry.is_finite()
            || !predicted.is_finite()
            || !filter_prediction.is_finite()
            || (self.corrections_enabled && !self.filter.predict(movement, seconds))
        {
            return Err(Fault::InvalidEstimate);
        }
        self.odometry = odometry;
        self.history.record(raw.time, odometry);
        let [lateral_velocity, forward_velocity, angular_velocity] =
            self.velocity.update(movement, seconds, rates);
        self.forward_velocity = forward_velocity;
        self.angular_velocity = angular_velocity;
        self.forward_travel += movement.translation.y;
        let mut reports = [None; MAX_RANGES];
        let mut aligned = [RangeSensorConfig::new(Pose::default()); MAX_RANGES];
        aligned[..self.sensors.len()].copy_from_slice(&self.sensors);
        let (checking_pose, active_ranges) = if self.reset.is_some() {
            (predicted, self.range_masks.reset())
        } else {
            (filter_prediction, self.filter_ranges)
        };
        if self.corrections_enabled && active_ranges != 0 {
            let range_tick = RangeTick {
                pose: checking_pose,
                now: raw.time,
            };
            for (index, sensor) in self.sensors.iter().copied().enumerate() {
                if active_ranges & (1 << index) == 0 {
                    continue;
                }
                let observation = observations.get(index).copied().flatten();
                // Duplicate packets are rejected by inspect regardless of alignment.
                // Avoid history search and two rotations at the faster odometry rate.
                let mount = observation
                    .filter(|sample| {
                        self.range_times[index].is_none_or(|last| sample.timestamp > last)
                    })
                    .and_then(|sample| {
                        self.history
                            .sensor_mount(sample.timestamp, odometry, sensor.mount)
                    });
                if let Some(mount) = mount {
                    aligned[index].mount = mount;
                }
                reports[index] = Some(inspect(
                    range_tick,
                    &self.map,
                    aligned[index],
                    observation,
                    &mut self.range_times[index],
                    mount.is_some(),
                ));
            }
        }
        let external = reports
            .iter()
            .flatten()
            .any(|report| report.constraint.is_ok());
        let mut correction = Correction::None;
        let mut candidate = None;
        let mut distance_reset = false;
        let mut distance_reset_axes = 0;
        let mut reset_status = DistanceReset::Idle;
        self.pose = predicted;
        self.filter_pose = filter_prediction;
        if let Some(reset) = &mut self.reset {
            match reset.update(
                ResetTick {
                    time: raw.time,
                    pose: predicted,
                    forward_velocity: self.forward_velocity,
                    lateral_velocity,
                    angular_velocity: self.angular_velocity,
                },
                &self.map,
                &self.sensors,
                &reports,
                self.dsr_config,
            ) {
                ResetResult::Waiting => reset_status = DistanceReset::Waiting,
                ResetResult::Applied(pose, axes) => {
                    let pose = self.contact.map_or(pose, |contact| contact.project(pose));
                    distance_reset_axes = axes;
                    reset_status = DistanceReset::Applied { axes };
                    self.reset = None;
                    self.filter.reset(pose);
                    self.filter.set_contact_constraint(self.contact);
                    self.pose = pose;
                    self.filter_pose = pose;
                    candidate = Some(pose);
                    distance_reset = true;
                    correction = Correction::Accepted;
                }
                ResetResult::Rejected => {
                    // A runner retries until its deadline. Retain the requested
                    // axis rank across rejected collections; its ordinary repeat
                    // request must not downgrade a full reset to one axis.
                    reset.restart();
                    reset_status = DistanceReset::Rejected;
                    correction = Correction::Rejected;
                }
            }
        } else if self.corrections_enabled {
            candidate = self.filter.observe(Observations {
                predicted: filter_prediction,
                sensors: &aligned[..self.sensors.len()],
                reports: &reports,
                map: &self.map,
            });
            if let Some(estimate) = candidate {
                let estimate = self
                    .contact
                    .map_or(estimate, |contact| contact.project(estimate));
                self.filter_pose = estimate;
                if self.config.shadow {
                    correction = Correction::Shadow;
                } else {
                    self.pose = estimate;
                    correction = Correction::Accepted;
                }
            } else if external && self.filter.corrects_pose() {
                correction = Correction::Rejected;
            }
        }
        Ok(Update {
            raw,
            odometry,
            predicted,
            pose: self.pose,
            candidate,
            correction,
            distance_reset,
            distance_reset_axes,
            reset: reset_status,
            forward_velocity: self.forward_velocity,
            lateral_velocity,
            angular_velocity: self.angular_velocity,
            ranges: reports,
        })
    }
}

impl Localizer<'_> {
    /// Switch estimator without changing the sensor model or claiming pose recovery.
    pub fn set_estimator(&mut self, estimator: Estimator, shadow: bool) -> bool {
        let Some(mut filter) = Filter::new(self.pose, estimator) else {
            return false;
        };
        self.filter_ranges = self.range_masks.for_roles(filter.range_roles());
        filter.set_contact_constraint(self.contact);
        self.filter = filter;
        self.filter_pose = self.pose;
        self.config.estimator = estimator;
        self.config.shadow = shadow;
        self.reset = None;
        true
    }
}

impl<E: PoseEstimator> TracksPosition for Localizer<'_, E> {
    fn position(&self) -> DVec2 {
        self.pose.position
    }
}
impl<E: PoseEstimator> TracksHeading for Localizer<'_, E> {
    fn heading(&self) -> Angle {
        self.pose.heading
    }
}
impl<E: PoseEstimator> TracksForwardTravel for Localizer<'_, E> {
    fn forward_travel(&self) -> f64 {
        self.forward_travel
    }
}
impl<E: PoseEstimator> TracksVelocity for Localizer<'_, E> {
    fn linear_velocity(&self) -> f64 {
        self.forward_velocity
    }
    fn angular_velocity(&self) -> f64 {
        self.angular_velocity
    }
}

#[cfg(test)]
mod tests;
