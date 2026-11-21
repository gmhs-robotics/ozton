//! One timestamped localization pipeline for a robot's motion and diagnostics.

use std::{borrow::Cow, time::Duration};

use glam::DVec2;
use ozton_math::{
    geometry::{Pose, wrap_heading},
    localization::range::Surface,
    uom::si::{
        angle::radian,
        f64::{Angle as UomAngle, Length, Time},
    },
};
use vexide::math::Angle;

use crate::{
    Tracking as TrackingSystem, TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity,
};

mod estimate;
mod ranges;
mod snapshot;
mod tracking;
pub use tracking::Tracking;
pub mod vex;

pub use estimate::{Ekf, EkfConfig, Estimator, Filter, Mcl, Observations, Odometry, PoseEstimator};
pub use ranges::{MAX_RANGES, RangeObservation, RangeReport, RangeSensorConfig};
use ranges::{RangeTick, ResetResult, ResetTick, ResetWindow, inspect};
pub use snapshot::{Fault, GpsFix, OdometryConfig, Snapshot};

/// Sensor recovery does not prove that the robot stayed at its old field position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PoseTrust {
    Unregistered,
    Trusted,
    NeedsRelocalization,
}

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub odometry: OdometryConfig,
    pub estimator: Estimator,
    /// Run the estimator for diagnostics while motion uses dead reckoning.
    pub shadow: bool,
    pub max_correction: Length,
}

impl Config {
    /// Creates localizer settings with a unit-bearing correction limit.
    pub const fn new(
        odometry: OdometryConfig,
        estimator: Estimator,
        shadow: bool,
        max_correction: Length,
    ) -> Self {
        Self {
            odometry,
            estimator,
            shadow,
            max_correction,
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
    range_times: [Option<Duration>; MAX_RANGES],
    gps_time: Option<Duration>,
    forward_travel: f64,
    forward_velocity: f64,
    angular_velocity: f64,
    reset: Option<ResetWindow>,
    trust: PoseTrust,
    corrections_enabled: bool,
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
        sensors: Cow<'a, [RangeSensorConfig]>,
    ) -> Option<Self> {
        if !config.odometry.is_valid()
            || !pose.is_finite()
            || !config.max_correction.value.is_finite()
            || config.max_correction.value <= 0.0
            || map.len() > 32
            || sensors.len() > MAX_RANGES
            || map.iter().any(|surface| !surface.is_valid())
            || sensors.iter().any(|sensor| !sensor.valid(&map))
        {
            return None;
        }
        let pose = Pose {
            heading: wrap_heading(UomAngle::new::<radian>(pose.heading)),
            ..pose
        };
        Some(Self {
            config,
            map,
            sensors,
            filter: Filter::new(pose, config.estimator)?,
            pose,
            odometry: pose,
            filter_pose: pose,
            previous: None,
            range_times: [None; MAX_RANGES],
            gps_time: None,
            forward_travel: 0.0,
            forward_velocity: 0.0,
            angular_velocity: 0.0,
            reset: None,
            trust: PoseTrust::Unregistered,
            corrections_enabled: true,
        })
    }
}

impl<'a, E: PoseEstimator> Localizer<'a, E> {
    fn with_filter<N: PoseEstimator>(self, mut filter: N) -> Localizer<'a, N> {
        filter.reset(self.pose);
        let Self {
            config,
            map,
            sensors,
            pose,
            odometry,
            previous,
            range_times,
            gps_time,
            forward_travel,
            forward_velocity,
            angular_velocity,
            trust,
            corrections_enabled,
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
            range_times,
            gps_time,
            forward_travel,
            forward_velocity,
            angular_velocity,
            reset: None,
            trust,
            corrections_enabled,
        }
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
            }
            self.corrections_enabled = enabled;
        }
    }

    /// Explicitly register a surveyed pose with set_pose after an outage.
    pub fn invalidate_pose(&mut self) {
        self.trust = PoseTrust::NeedsRelocalization;
        self.previous = None;
        self.reset = None;
        self.forward_velocity = 0.0;
        self.angular_velocity = 0.0;
    }

    pub fn pose(&self) -> Pose {
        self.pose
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
            heading: wrap_heading(UomAngle::new::<radian>(pose.heading)),
            ..pose
        };
        self.trust = PoseTrust::Trusted;
        self.pose = pose;
        self.odometry = pose;
        self.filter_pose = pose;
        self.filter.reset(pose);
        self.previous = None;
        self.forward_velocity = 0.0;
        self.angular_velocity = 0.0;
        self.reset = None;
        true
    }

    /// Attempt an immediate geometric DSR on the next fresh sample.
    pub fn request_distance_reset(&mut self) {
        if self.corrections_enabled {
            self.reset.get_or_insert_with(ResetWindow::default);
        }
    }

    /// Request one or two independently observed translation axes. Requiring two
    /// waits for both, instead of publishing an earlier one-axis partial reset.
    /// A repeated ordinary reset request does not weaken this requirement.
    /// Rejected collections retry at rest until accepted or explicitly canceled;
    /// the caller must enforce a timeout, as the autonomous runner does.
    pub fn request_distance_reset_axes(&mut self, minimum_axes: u8) -> bool {
        if !self.corrections_enabled || !(1..=2).contains(&minimum_axes) {
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

    pub fn update(
        &mut self,
        raw: Snapshot,
        observations: &[Option<RangeObservation>],
    ) -> Result<Update, Fault> {
        let result = self.update_inner(raw, observations);
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
    ) -> Result<Update, Fault> {
        if self.previous.is_some_and(|last| raw.time <= last.time) {
            return Err(Fault::Time);
        }
        let previous = self.previous.replace(raw).ok_or(Fault::Initializing)?;
        let (movement, dt) = match snapshot::delta(previous, raw, self.config.odometry) {
            Ok(sample) => sample,
            Err(fault) => {
                self.forward_velocity = 0.0;
                self.angular_velocity = 0.0;
                return Err(fault);
            }
        };
        let seconds = dt.as_secs_f64();
        let odometry = self.odometry.integrate(movement);
        let predicted = if self.pose == self.odometry {
            odometry
        } else {
            self.pose.integrate(movement)
        };
        // Odometry and an uncorrected filter share the same pose in the common
        // path. Reuse integration rather than evaluating the same trig again.
        let filter_prediction = if !self.corrections_enabled || self.filter_pose == self.pose {
            predicted
        } else {
            self.filter_pose.integrate(movement)
        };
        if !odometry.is_finite()
            || !predicted.is_finite()
            || !filter_prediction.is_finite()
            || (self.corrections_enabled && !self.filter.predict(movement, seconds))
        {
            return Err(Fault::InvalidEstimate);
        }
        self.odometry = odometry;
        self.forward_velocity = movement.translation.y / seconds;
        self.angular_velocity = movement.heading / seconds;
        self.forward_travel += movement.translation.y;
        let mut reports = [None; MAX_RANGES];
        let checking_pose = if self.reset.is_some() {
            predicted
        } else {
            filter_prediction
        };
        for (index, sensor) in
            self.sensors
                .iter()
                .copied()
                .enumerate()
                .take(if self.corrections_enabled {
                    self.sensors.len()
                } else {
                    0
                })
        {
            reports[index] = Some(inspect(
                RangeTick {
                    pose: checking_pose,
                    movement,
                    dt,
                    now: raw.time,
                },
                &self.map,
                sensor,
                observations.get(index).copied().flatten(),
                &mut self.range_times[index],
            ));
        }
        let gps = raw.gps.filter(|fix| {
            if !self.corrections_enabled {
                return false;
            }
            let fresh = self
                .gps_time
                .is_none_or(|previous| fix.timestamp > previous)
                && raw.time.checked_sub(fix.timestamp).is_some_and(|age| {
                    age <= Duration::from_millis(100)
                        && self.forward_velocity.abs() * age.as_secs_f64() < 0.03
                });
            if fresh {
                self.gps_time = Some(fix.timestamp);
            }
            fresh
        });
        let external = gps.is_some()
            || reports
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
                    lateral_velocity: movement.translation.x / seconds,
                    angular_velocity: self.angular_velocity,
                },
                &self.map,
                &self.sensors,
                &reports,
                self.config.max_correction.value,
            ) {
                ResetResult::Waiting => reset_status = DistanceReset::Waiting,
                ResetResult::Applied(pose, axes) => {
                    distance_reset_axes = axes;
                    reset_status = DistanceReset::Applied { axes };
                    self.reset = None;
                    self.filter.reset(pose);
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
                gps,
                sensors: &self.sensors,
                reports: &reports,
                map: &self.map,
                maximum_correction: self.config.max_correction.value,
            });
            if let Some(estimate) = candidate {
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
            lateral_velocity: movement.translation.x / seconds,
            angular_velocity: self.angular_velocity,
            ranges: reports,
        })
    }
}

impl Localizer<'_> {
    /// Switch estimator without changing the sensor model or claiming pose recovery.
    pub fn set_estimator(&mut self, estimator: Estimator, shadow: bool) -> bool {
        let Some(filter) = Filter::new(self.pose, estimator) else {
            return false;
        };
        self.filter = filter;
        self.filter_pose = self.pose;
        self.config.estimator = estimator;
        self.config.shadow = shadow;
        self.reset = None;
        true
    }
}

impl<E: PoseEstimator> TrackingSystem for Localizer<'_, E> {}
impl<E: PoseEstimator> TracksPosition for Localizer<'_, E> {
    fn position(&self) -> DVec2 {
        self.pose.position
    }
}
impl<E: PoseEstimator> TracksHeading for Localizer<'_, E> {
    fn heading(&self) -> Angle {
        Angle::from_radians(self.pose.heading)
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
