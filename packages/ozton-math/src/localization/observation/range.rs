//! Distance sensor geometry and timestamped observations.

use core::time::Duration;

use crate::{
    geometry::Pose,
    localization::range::{RangeGate, RangeReading, Rejection, Surface},
    units::Vec3,
    uom::{
        ConstZero,
        si::{
            f64::{Angle, Length, Time},
            time::second,
        },
    },
};

pub const MAX_RANGES: usize = 8;

/// Published V5 Distance Sensor (276-4852) limits.
/// https://kb.vex.com/hc/en-us/articles/360050696511-Using-the-Distance-Sensor-with-VEX-V5
pub const V5_MIN_RANGE: Length = crate::qty!(20 mm);
pub const V5_MAX_RANGE: Length = crate::qty!(2000 mm);

/// Conservative continuous envelope of the published approximate accuracy:
/// 15 mm below 200 mm, 5% above. The 15 mm floor avoids a discontinuity
/// and does not imply that the manufacturer specifies Gaussian statistics.
pub fn v5_range_accuracy(distance: Length) -> Length {
    crate::qty!((0.015_f64.max(0.05 * distance.value)) m)
}

#[derive(Debug, Clone, Copy)]
pub struct RangeSensorConfig {
    pub mount: Pose,
    /// Beam height in the same chassis frame as the planar mount.
    pub mount_height: Length,
    pub gate: RangeGate,
    pub bias: Length,
    pub sigma: Length,
    pub surface: Option<usize>,
    /// Eligible map surfaces, e.g. walls reaching this sensor's mounting height.
    pub surface_mask: u128,
}

impl RangeSensorConfig {
    /// Default V5 range gates, no bias, and 25 mm observation noise.
    pub const fn new(mount: Pose) -> Self {
        Self {
            mount,
            mount_height: Length::ZERO,
            gate: RangeGate::DEFAULT,
            bias: Length::ZERO,
            sigma: crate::qty!(0.025 m),
            surface: None,
            surface_mask: u128::MAX,
        }
    }

    /// Configure a horizontal beam from its chassis position and bearing.
    pub const fn at_mount((position, bearing): (Vec3<Length>, Angle)) -> Self {
        Self {
            mount_height: position.up,
            ..Self::new(Pose::new(
                crate::geometry::point(position.right, position.forward),
                bearing,
            ))
        }
    }

    pub fn valid(self, map: &[Surface]) -> bool {
        self.mount.is_finite()
            && self.mount_height.value.is_finite()
            && self.gate.is_valid()
            && self.effective_gate().is_valid()
            && self.bias.value.is_finite()
            && self.sigma.value.is_finite()
            && self.sigma.value > 0.0
            && self.surface.is_none_or(|index| index < map.len())
    }

    /// Intersect explicit surface selection with the surveyed horizontal beam
    /// height. Bits beyond the actual map are removed as well.
    pub fn with_visible_surfaces(mut self, map: &[Surface]) -> Self {
        let visible = map.iter().take(128).enumerate().fold(0u128, |mask, (index, face)| {
            mask | (u128::from(face.visible_at(self.mount_height)) << index)
        });
        self.surface_mask &= visible;
        self
    }

    /// Apply the physical sensor limits even if the application supplies
    /// a wider acceptance gate.
    pub fn effective_gate(self) -> RangeGate {
        let mut gate = self.gate;
        gate.min_range = if gate.min_range < V5_MIN_RANGE {
            V5_MIN_RANGE
        } else {
            gate.min_range
        };
        gate.max_range = if gate.max_range > V5_MAX_RANGE {
            V5_MAX_RANGE
        } else {
            gate.max_range
        };
        gate
    }

    /// Limit the trusted range by an application-selected accuracy budget.
    /// For example, 50 mm permits returns up to 1 m. The existing gate can
    /// be stricter. Returns false for an impossible or invalid budget.
    pub fn limit_accuracy(&mut self, maximum_error: Length) -> bool {
        if !maximum_error.value.is_finite() || maximum_error.value < 0.015 {
            return false;
        }
        let maximum = crate::qty!((maximum_error.value / 0.05) m);
        let mut gate = self.effective_gate();
        gate.max_range = if gate.max_range > maximum {
            maximum
        } else {
            gate.max_range
        };
        if !gate.is_valid() {
            return false;
        }
        self.gate = gate;
        true
    }

    /// Effective range standard deviation used as a conservative engineering
    /// noise model, not a manufacturer-specified statistical confidence bound.
    pub fn range_sigma(self, distance: Length) -> f64 {
        self.sigma.value.max(v5_range_accuracy(distance).value)
    }

    /// Use the farther of the measured and associated modeled distances,
    /// then project range uncertainty onto the surface normal.
    pub fn constraint_sigma(
        self,
        constraint: crate::localization::range::Constraint,
        distance: f64,
    ) -> f64 {
        self.range_sigma(crate::qty!((distance.max(constraint.hit.distance)) m))
            * constraint.hit.incidence.abs()
    }

    /// Validate raw hardware range before any bias correction. PROS documents
    /// the unavailable confidence sentinel (10/63) for short-range readings.
    /// Bypass only that sentinel, not a reported low confidence at long range.
    pub fn sample_gate(self, sample: RangeObservation) -> Result<RangeGate, Rejection> {
        if sample.occluded {
            return Err(Rejection::Occluded);
        }
        if !(V5_MIN_RANGE..=V5_MAX_RANGE).contains(&sample.distance) {
            return Err(Rejection::OutOfRange);
        }
        let mut gate = self.effective_gate();
        if sample.distance.value <= 0.2 && (sample.confidence - 10.0 / 63.0).abs() < 1e-6 {
            gate.min_confidence = 0.0;
        }
        Ok(gate)
    }

    /// A sensor-quality check for relative alignment. Pose association and
    /// innovation limits apply separately to localization corrections.
    pub fn relative_distance(
        self,
        sample: RangeObservation,
        now: Duration,
    ) -> Result<Length, Rejection> {
        let age = now.checked_sub(sample.timestamp).ok_or(Rejection::Stale)?;
        let reading = self.reading(sample, Time::new::<second>(age.as_secs_f64()));
        reading.validate(self.sample_gate(sample)?)?;
        Ok(reading.distance)
    }

    pub fn reading(self, sample: RangeObservation, age: Time) -> RangeReading {
        RangeReading {
            distance: sample.distance - self.bias,
            confidence: sample.confidence,
            age,
            occluded: sample.occluded,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct RangeObservation {
    pub timestamp: Duration,
    pub distance: Length,
    pub confidence: f64,
    pub occluded: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn v5_range_limits_and_accuracy_budget() {
        let mut sensor = RangeSensorConfig::new(Pose::default());
        sensor.gate.min_range = crate::qty!(0 mm);
        sensor.gate.max_range = crate::qty!(3 m);
        assert_eq!(sensor.effective_gate().min_range, V5_MIN_RANGE);
        assert_eq!(sensor.effective_gate().max_range, V5_MAX_RANGE);
        assert_eq!(sensor.range_sigma(crate::qty!(100 mm)), 0.025);
        assert!((sensor.range_sigma(crate::qty!(2 m)) - 0.1).abs() < 1e-12);
        assert!(sensor.limit_accuracy(crate::qty!(50 mm)));
        assert!((sensor.gate.max_range.value - 1.0).abs() < 1e-12);
        assert!(!sensor.limit_accuracy(crate::qty!(10 mm)));
    }

    #[test]
    fn short_range_confidence_sentinel_and_raw_range() {
        let sensor = RangeSensorConfig::new(Pose::default());
        let mut sample = RangeObservation {
            timestamp: Duration::ZERO,
            distance: crate::qty!(100 mm),
            confidence: 10.0 / 63.0,
            occluded: false,
        };
        assert!(sensor.relative_distance(sample, Duration::ZERO).is_ok());
        sample.distance = crate::qty!(500 mm);
        assert_eq!(
            sensor.relative_distance(sample, Duration::ZERO),
            Err(Rejection::LowConfidence)
        );
        sample.distance = crate::qty!(2001 mm);
        assert_eq!(
            sensor.relative_distance(sample, Duration::ZERO),
            Err(Rejection::OutOfRange)
        );
    }
}
