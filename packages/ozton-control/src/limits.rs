//! Finite travel and arrival limits for position-controlled mechanisms.

use ozton_math::{
    si,
    uom::si::{Dimension, Quantity, SI},
};

use crate::axis::{AxisDimension, Position, Rotary};

/// Inclusive travel bounds and an absolute arrival tolerance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionLimits<D: AxisDimension = Rotary> {
    pub travel: Option<(Position<D>, Position<D>)>,
    pub arrival_tolerance: Position<D>,
}

impl<D: AxisDimension> PositionLimits<D> {
    pub const fn new(
        travel: Option<(Position<D>, Position<D>)>,
        arrival_tolerance: Position<D>,
    ) -> Self {
        Self {
            travel,
            arrival_tolerance,
        }
    }

    pub const fn valid(&self) -> bool {
        valid_range(self.travel)
            && self.arrival_tolerance.value.is_finite()
            && self.arrival_tolerance.value >= 0.0
    }

    /// Clamps a target to the configured travel.
    pub fn target(&self, target: Position<D>) -> Position<D> {
        clamp_target(target, self.travel)
    }

    /// Checks arrival against the clamped target. Invalid inputs fail closed.
    pub fn ready(&self, position: Position<D>, target: Position<D>) -> bool {
        self.valid()
            && position.value.is_finite()
            && (position.value - self.target(target).value).abs() <= self.arrival_tolerance.value
    }
}

type Bounds<D> = (Quantity<D, SI<f64>, f64>, Quantity<D, SI<f64>, f64>);

pub const fn valid_range<D: Dimension + ?Sized>(range: Option<Bounds<D>>) -> bool {
    match range {
        Some((min, max)) => min.value.is_finite() && max.value.is_finite() && min.value < max.value,
        None => true,
    }
}

pub fn clamp_target<D: Dimension + ?Sized>(
    target: Quantity<D, SI<f64>, f64>,
    range: Option<Bounds<D>>,
) -> Quantity<D, SI<f64>, f64> {
    if !target.value.is_finite() {
        return target;
    }
    match range {
        Some((min, max)) if valid_range(range) => si(target.value.clamp(min.value, max.value)),
        Some(_) => si(f64::NAN),
        None => target,
    }
}
