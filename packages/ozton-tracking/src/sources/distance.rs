//! Distance sensor geometry and timestamped observations.

use std::time::Duration;

use ozton_math::{
    localization::range::{Mount, RangeGate, RangeReading, Rejection, Surface},
    uom::{
        ConstZero,
        si::{
            f64::{Angle, Length, Time},
            time::second,
        },
    },
};

use crate::mount::Vec3;

pub const MAX_RANGES: usize = 8;

#[derive(Debug, Clone, Copy)]
pub struct RangeSensorConfig {
    pub mount: Mount,
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
    pub const fn new(mount: Mount) -> Self {
        Self {
            mount,
            mount_height: Length::ZERO,
            gate: RangeGate::DEFAULT,
            bias: Length::ZERO,
            sigma: Length {
                value: 0.025,
                ..Length::ZERO
            },
            surface: None,
            surface_mask: u128::MAX,
        }
    }

    /// Configure a horizontal beam from its chassis position and bearing.
    pub const fn at_mount(position: Vec3<Length>, bearing: Angle) -> Self {
        Self {
            mount_height: position.up,
            ..Self::new(Mount::new(position.right, position.forward, bearing))
        }
    }

    pub fn valid(self, map: &[Surface]) -> bool {
        self.mount.offset.is_finite()
            && self.mount.heading.is_finite()
            && self.mount_height.value.is_finite()
            && self.gate.is_valid()
            && self.bias.value.is_finite()
            && self.sigma.value.is_finite()
            && self.sigma.value > 0.0
            && self.surface.is_none_or(|index| index < map.len())
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
        reading.validate(self.gate)?;
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
