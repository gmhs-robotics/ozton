//! Finite travel and arrival limits for position-controlled mechanisms.

/// Optional inclusive travel and absolute arrival tolerance in the caller's position units.
///
/// Use a constant plus a const assertion on [`Self::valid`] for static settings.
/// Runtime commands can validate the same value before accepting it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionLimits<T = f64> {
    pub travel: Option<(T, T)>,
    pub arrival_tolerance: T,
}

impl<T> PositionLimits<T> {
    /// Defines the travel range and maximum absolute error for arrival.
    pub const fn new(travel: Option<(T, T)>, arrival_tolerance: T) -> Self {
        Self {
            travel,
            arrival_tolerance,
        }
    }
}

impl PositionLimits<vexide::math::Angle> {
    /// Converts typed angles for controllers operating in accumulated encoder degrees.
    pub const fn in_degrees(self) -> PositionLimits<f64> {
        let travel = match self.travel {
            Some((min, max)) => Some((min.as_degrees(), max.as_degrees())),
            None => None,
        };
        PositionLimits::new(travel, self.arrival_tolerance.as_degrees())
    }
}

impl PositionLimits<f64> {
    pub const fn valid(self) -> bool {
        valid_range(self.travel)
            && self.arrival_tolerance.is_finite()
            && self.arrival_tolerance >= 0.0
    }

    /// Clamp to travel while keeping non-finite targets invalid.
    pub fn target(self, target: f64) -> f64 {
        clamp_target(target, self.travel)
    }

    /// Checks arrival against the clamped target. Invalid inputs fail closed.
    pub fn ready(self, position: f64, target: f64) -> bool {
        self.valid()
            && position.is_finite()
            && (position - self.target(target)).abs() <= self.arrival_tolerance
    }
}

/// Validate an optional inclusive travel range. None means unrestricted.
pub const fn valid_range(range: Option<(f64, f64)>) -> bool {
    match range {
        Some((min, max)) => min.is_finite() && max.is_finite() && min < max,
        None => true,
    }
}

/// Clamp a finite target; invalid raw targets remain invalid for caller validation.
pub fn clamp_target(target: f64, range: Option<(f64, f64)>) -> f64 {
    if !target.is_finite() {
        return target;
    }
    match range {
        Some((min, max)) if valid_range(range) => target.clamp(min, max),
        Some(_) => f64::NAN,
        None => target,
    }
}
