//! Conditional Settling Utility for Motion Algorithms
//!
//! This module provides utilities for determining when a control system
//! has "settled" at its target state, signaling that the algorithm has successfully reached its goal.
//! While different control algorithms may define "completeness" in varying ways, this module's
//! [`Tolerances`] struct implements the common approach of settling via *tolerances*.
//!
//! # Tolerances
//!
//! *Tolerances* define an acceptable range of values around a target state. This is useful to define,
//! because physical systems will never achieve their exact target due to real-world factors
//! like friction, sensor noise, and mechanical limitations. Under the [`Tolerances`] struct, a system is considered
//! "settled" when it meets specified error and velocity tolerances for a given duration.

use std::time::{Duration, Instant};

use crate::axis::{AxisDimension, Position, Rotary, Velocity};

/// Describes when a control system has stabilized reasonably near its setpoint.
///
/// This struct monitors both position error and velocity to determine if a system has
/// reached and stabilized at its target. It can be configured with tolerances for both
/// error and velocity, a required duration to maintain those tolerances.
///
/// # Settling Logic
///
/// A system is considered settled if either:
/// - The specified timeout has elapsed since the first call to [`Tolerances::check`], OR
/// - Both:
///   1. The error and velocity are within their respective tolerances.
///   2. The system has maintained these tolerances for the specified duration.
///
/// If the system leaves the tolerance window before the duration is met, the tolerance timer resets.
#[derive(Debug, Copy, Clone, PartialEq, PartialOrd)]
pub struct Tolerances<D: AxisDimension = Rotary> {
    tolerance_timestamp: Option<Instant>,

    /// Duration for which `error_tolerance` and `velocity_tolerance` must be satisfied.
    pub duration: Option<Duration>,

    /// Minimum error range.
    pub error_tolerance: Option<Position<D>>,

    /// Minimum velocity range.
    pub velocity_tolerance: Option<Velocity<D>>,
}

impl<D: AxisDimension> Default for Tolerances<D> {
    fn default() -> Self {
        Self::new()
    }
}

impl<D: AxisDimension> Tolerances<D> {
    /// Creates a new [`Tolerances`] instance with no configured tolerances or timings.
    ///
    /// Until tolerances are configured using the builder methods, all tolerance
    /// checks will pass immediately.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            tolerance_timestamp: None,
            duration: None,
            error_tolerance: None,
            velocity_tolerance: None,
        }
    }

    /// Sets the maximum acceptable error value for settling.
    ///
    /// The error tolerance defines how close to the target position the system
    /// must be to be considered "within tolerance".
    #[must_use]
    pub const fn error(mut self, tolerance: Position<D>) -> Self {
        self.error_tolerance = Some(tolerance);
        self
    }

    /// Sets the maximum acceptable velocity for settling.
    ///
    /// The velocity tolerance defines how slow the system must be moving to be
    /// considered "stable".
    #[must_use]
    pub const fn velocity(mut self, tolerance: Velocity<D>) -> Self {
        self.velocity_tolerance = Some(tolerance);
        self
    }

    /// Sets how long the system must remain within tolerances to be considered settled.
    ///
    /// This duration acts as a "debounce" to ensure the system has truly stabilized
    /// and isn't just passing through the tolerance window momentarily.
    #[must_use]
    pub const fn duration(mut self, duration: Duration) -> Self {
        self.duration = Some(duration);
        self
    }

    /// Check a sample using state owned by the caller.
    pub fn check_with(
        &self,
        state: &mut Settling,
        error: Position<D>,
        velocity: Velocity<D>,
        now: Instant,
    ) -> bool {
        let in_tolerances = error.value.is_finite()
            && velocity.value.is_finite()
            && self
                .error_tolerance
                .is_none_or(|tolerance| error.value.abs() < tolerance.value)
            && self
                .velocity_tolerance
                .is_none_or(|tolerance| velocity.value.abs() < tolerance.value);
        if !in_tolerances {
            *state = Settling::default();
            return false;
        }
        let since = *state.since.get_or_insert(now);
        self.duration
            .is_none_or(|duration| now.saturating_duration_since(since) >= duration)
    }

    /// Check a sample with the legacy standalone timer.
    pub fn check(&mut self, error: Position<D>, velocity: Velocity<D>) -> bool {
        let mut state = Settling {
            since: self.tolerance_timestamp,
        };
        let settled = self.check_with(&mut state, error, velocity, Instant::now());
        self.tolerance_timestamp = state.since;
        settled
    }
}

/// Tracks how long one command has remained continuously within tolerance.
#[derive(Debug, Default)]
pub struct Settling {
    since: Option<Instant>,
}
