//! One timestamped localization pipeline for a robot's motion and diagnostics.

use std::{borrow::Cow, time::Duration};

use glam::DVec2;
use ozton_math::{
    geometry::{Pose, wrap_heading},
    localization::range::Surface,
};
use vexide::math::Angle;

use crate::{Tracking, TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity};

mod estimate;
mod ranges;
mod snapshot;
pub mod vex;

use estimate::Filter;
pub use estimate::{EkfConfig, Estimator};
pub use ranges::{MAX_RANGES, RangeObservation, RangeReport, RangeSensorConfig};
use ranges::{ResetResult, ResetWindow, inspect};
pub use snapshot::{Fault, GpsFix, OdometryConfig, Snapshot};

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub odometry: OdometryConfig,
    pub estimator: Estimator,
    /// Run the estimator for diagnostics while motion uses dead reckoning.
    pub shadow: bool,
    pub max_correction_meters: f64,
}

#[derive(Debug, Clone, Copy)]
pub enum Correction {
    None,
    Accepted,
    Rejected,
    Shadow,
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
    pub forward_velocity: f64,
    pub angular_velocity: f64,
    pub ranges: [Option<RangeReport>; MAX_RANGES],
}

/// Composes independent sensors without observing the same packet twice.
pub struct Localizer<'a> {
    config: Config,
    map: Cow<'a, [Surface]>,
    sensors: Cow<'a, [RangeSensorConfig]>,
    filter: Filter,
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
            || !config.max_correction_meters.is_finite()
            || config.max_correction_meters <= 0.0
            || map.len() > 32
            || sensors.len() > MAX_RANGES
            || map.iter().any(|surface| !surface.is_valid())
            || sensors.iter().any(|sensor| !sensor.valid(&map))
        {
            return None;
        }
        let pose = Pose {
            heading: wrap_heading(pose.heading),
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
        })
    }

    pub fn pose(&self) -> Pose {
        self.pose
    }

    pub fn estimator_name(&self) -> &'static str {
        match (self.config.estimator, self.config.shadow) {
            (Estimator::Odometry, _) => "Odometry",
            (Estimator::Ekf(_), true) => "Odometry / EKF shadow",
            (Estimator::Mcl(_), true) => "Odometry / MCL shadow",
            (Estimator::Ekf(_), false) => "EKF",
            (Estimator::Mcl(_), false) => "MCL",
        }
    }

    pub fn range_max_age(&self, index: usize) -> Option<f64> {
        self.sensors.get(index).map(|sensor| sensor.gate.max_age)
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
        self.reset.get_or_insert_with(ResetWindow::default);
    }

    pub fn update(
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
        let predicted = self.pose.integrate(movement);
        let filter_prediction = self.filter_pose.integrate(movement);
        if !odometry.is_finite()
            || !predicted.is_finite()
            || !filter_prediction.is_finite()
            || !self.filter.predict(movement, seconds)
        {
            return Err(Fault::InvalidEstimate);
        }
        self.odometry = odometry;
        self.forward_velocity = movement.translation.x / seconds;
        self.angular_velocity = movement.heading / seconds;
        self.forward_travel += movement.translation.x;
        let mut reports = [None; MAX_RANGES];
        let checking_pose = if self.reset.is_some() {
            predicted
        } else {
            filter_prediction
        };
        for (index, sensor) in self.sensors.iter().copied().enumerate() {
            reports[index] = Some(inspect(
                checking_pose,
                movement,
                dt,
                raw.time,
                &self.map,
                sensor,
                observations.get(index).copied().flatten(),
                &mut self.range_times[index],
            ));
        }
        let gps = raw.gps.filter(|fix| {
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
        self.pose = predicted;
        self.filter_pose = filter_prediction;
        if let Some(reset) = &mut self.reset {
            match reset.update(
                raw.time,
                predicted,
                &self.map,
                &self.sensors,
                &reports,
                self.config.max_correction_meters,
            ) {
                ResetResult::Waiting => {}
                ResetResult::Applied(pose) => {
                    self.reset = None;
                    self.filter.reset(pose);
                    self.pose = pose;
                    self.filter_pose = pose;
                    candidate = Some(pose);
                    distance_reset = true;
                    correction = Correction::Accepted;
                }
                ResetResult::Rejected => {
                    self.reset = None;
                    correction = Correction::Rejected;
                }
            }
        } else {
            candidate = self.filter.observe(
                filter_prediction,
                gps,
                &self.sensors,
                &reports,
                &self.map,
                self.config.max_correction_meters,
            );
            if let Some(estimate) = candidate {
                self.filter_pose = estimate;
                if self.config.shadow {
                    correction = Correction::Shadow;
                } else {
                    self.pose = estimate;
                    correction = Correction::Accepted;
                }
            } else if external && !matches!(self.config.estimator, Estimator::Odometry) {
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
            forward_velocity: self.forward_velocity,
            angular_velocity: self.angular_velocity,
            ranges: reports,
        })
    }
}

impl Tracking for Localizer<'_> {}
impl TracksPosition for Localizer<'_> {
    fn position(&self) -> DVec2 {
        self.pose.position
    }
}
impl TracksHeading for Localizer<'_> {
    fn heading(&self) -> Angle {
        Angle::from_radians(self.pose.heading)
    }
}
impl TracksForwardTravel for Localizer<'_> {
    fn forward_travel(&self) -> f64 {
        self.forward_travel
    }
}
impl TracksVelocity for Localizer<'_> {
    fn linear_velocity(&self) -> f64 {
        self.forward_velocity
    }
    fn angular_velocity(&self) -> f64 {
        self.angular_velocity
    }
}
