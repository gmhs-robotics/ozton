//! Estimator inputs and dispatch for applications that switch algorithms at runtime.

use ozton_math::{
    geometry::{BodyDelta, Pose},
    localization::{
        mcl::ParticleConfig,
        range::{Constraint, Surface},
    },
};

use super::{GpsFix, MAX_RANGES, RangeReport, RangeSensorConfig};

mod ekf;
mod mcl;
pub use ekf::{Ekf, EkfConfig};
pub use mcl::Mcl;

#[derive(Debug, Clone, Copy)]
pub enum Estimator {
    Odometry,
    Ekf(EkfConfig),
    Mcl(ParticleConfig),
}

/// Inputs shared by estimators after timestamp and range association checks.
pub struct Observations<'a> {
    pub predicted: Pose,
    pub gps: Option<GpsFix>,
    pub sensors: &'a [RangeSensorConfig],
    pub reports: &'a [Option<RangeReport>; MAX_RANGES],
    pub map: &'a [Surface],
    pub maximum_correction: f64,
}

impl<'a> Observations<'a> {
    fn measurements(
        &self,
    ) -> impl Iterator<Item = (&'a RangeSensorConfig, Constraint, f64)> + Clone {
        let constraints = self
            .sensors
            .iter()
            .zip(self.reports)
            .filter_map(|(sensor, report)| {
                let constraint = report.as_ref()?.constraint.ok()?;
                Some((
                    constraint,
                    sensor.sigma.value * constraint.hit.incidence.abs(),
                ))
            });
        let mut buffer = [(Constraint::default(), 0.0); MAX_RANGES];
        let mut count = 0;
        for constraint in constraints.take(MAX_RANGES) {
            buffer[count] = constraint;
            count += 1;
        }
        let consistent =
            ozton_math::localization::range::fit_translation(&buffer[..count]).is_some();
        self.sensors
            .iter()
            .zip(self.reports)
            .filter_map(move |(sensor, report)| {
                if !consistent {
                    return None;
                }
                let report = report.as_ref()?;
                Some((
                    sensor,
                    report.constraint.ok()?,
                    report.calibrated_distance?.value,
                ))
            })
    }
}

/// An independent pose correction algorithm. The localizer handles sensor timing.
pub trait PoseEstimator {
    fn name(&self, shadow: bool) -> &'static str;
    fn corrects_pose(&self) -> bool {
        true
    }
    fn reset(&mut self, pose: Pose);
    fn predict(&mut self, delta: BodyDelta, seconds: f64) -> bool;
    fn observe(&mut self, observations: Observations<'_>) -> Option<Pose>;
}

/// Wheel and heading dead reckoning with no automatic correction.
#[derive(Debug, Default, Clone, Copy)]
pub struct Odometry;

impl PoseEstimator for Odometry {
    fn name(&self, _: bool) -> &'static str {
        "Odometry"
    }
    fn corrects_pose(&self) -> bool {
        false
    }
    fn reset(&mut self, _: Pose) {}
    fn predict(&mut self, _: BodyDelta, _: f64) -> bool {
        true
    }
    fn observe(&mut self, _: Observations<'_>) -> Option<Pose> {
        None
    }
}

/// Runtime selection for applications whose routes change algorithms after construction.
#[doc(hidden)]
pub enum Filter {
    Odometry(Odometry),
    Ekf(Ekf),
    Mcl(Mcl),
}

impl Filter {
    pub(super) fn new(pose: Pose, estimator: Estimator) -> Option<Self> {
        match estimator {
            Estimator::Odometry => Some(Self::Odometry(Odometry)),
            Estimator::Ekf(config) => Ekf::new(pose, config).map(Self::Ekf),
            Estimator::Mcl(config) => Mcl::new(pose, config).map(Self::Mcl),
        }
    }
}

macro_rules! dispatch {
    ($self:expr, $method:ident($($arg:expr),*)) => {
        match $self {
            Filter::Odometry(inner) => inner.$method($($arg),*),
            Filter::Ekf(inner) => inner.$method($($arg),*),
            Filter::Mcl(inner) => inner.$method($($arg),*),
        }
    };
}

impl PoseEstimator for Filter {
    fn name(&self, shadow: bool) -> &'static str {
        dispatch!(self, name(shadow))
    }
    fn corrects_pose(&self) -> bool {
        !matches!(self, Self::Odometry(_))
    }
    fn reset(&mut self, pose: Pose) {
        dispatch!(self, reset(pose));
    }
    fn predict(&mut self, delta: BodyDelta, seconds: f64) -> bool {
        dispatch!(self, predict(delta, seconds))
    }
    fn observe(&mut self, observations: Observations<'_>) -> Option<Pose> {
        dispatch!(self, observe(observations))
    }
}
