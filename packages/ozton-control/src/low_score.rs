//! Reusable scoring preset. Lower the lift, eject, and restore the captured pose.

use std::time::Duration;

use ozton_math::uom::si::f64::Angle;

use crate::limits::PositionLimits;

/// Motion limits and timing for a low scoring preset.
#[derive(Clone, Copy)]
pub struct LowScoreLimits {
    pub lift: PositionLimits,
    pub pivot: PositionLimits,
    pub low_height: Angle,
    pub eject_time: Duration,
    pub travel_timeout: Duration,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    Lower,
    Eject,
    Return,
}

struct Cycle {
    phase: Phase,
    elapsed: Duration,
    original_height: Angle,
    low_height: Angle,
    pivot_angle: Angle,
}

/// Why this robot's B cycle was cancelled before completion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LowScoreError {
    InvalidFeedback,
    TravelTimeout,
}

pub struct LowScoreCommand {
    pub lift_target: Angle,
    /// Hold the arrived bottom pose while the intake ejects.
    pub hold_lift: bool,
    pub pivot_target: Angle,
    pub intake_direction: i8,
}

/// Selects the automatic preset or manual control for this tick.
pub enum DriverScoring {
    Automatic,
    Manual,
    CancelAndHoldPivot,
}

#[derive(Default)]
pub struct LowScore {
    pressed: bool,
    requested: bool,
    cycle: Option<Cycle>,
}

impl LowScore {
    pub fn phase(&self) -> &'static str {
        match self.cycle.as_ref().map(|cycle| cycle.phase) {
            Some(Phase::Lower) => "lower",
            Some(Phase::Eject) => "eject",
            Some(Phase::Return) => "return",
            None => "idle",
        }
    }

    pub fn is_active(&self) -> bool {
        self.cycle.is_some()
    }

    /// Queue one cycle for the next mechanism tick.
    pub fn request(&mut self) {
        if !self.is_active() {
            self.requested = true;
        }
    }

    pub fn ready_to_start(&self, button: bool) -> bool {
        button && !self.pressed && !self.is_active() && !self.requested
    }

    /// Manual input and a second press preempt automatic scoring.
    pub fn driver_command(&self, button: bool, manual: bool) -> DriverScoring {
        if !manual && self.needs_update(button) {
            DriverScoring::Automatic
        } else if self.is_active() {
            DriverScoring::CancelAndHoldPivot
        } else {
            DriverScoring::Manual
        }
    }

    /// A second press leaves automatic control immediately, before sensor reads
    /// or automatic output writes. The Driver caller then cancels the cycle.
    pub fn needs_update(&self, button: bool) -> bool {
        let new_press = button && !self.pressed;
        if self.is_active() {
            !new_press
        } else {
            self.requested || new_press
        }
    }

    /// Manual takeover consumes a held B so releasing a manual button cannot
    /// unexpectedly start scoring. A new cycle requires release and re-press.
    pub fn cancel_driver(&mut self, button: bool) {
        self.cancel();
        self.pressed = button;
    }

    pub fn cancel(&mut self) {
        self.requested = false;
        self.cycle = None;
    }

    /// Heights are positive upward; pivot angles are positive toward pickup.
    /// An error cancels timed-out travel, allowing the caller to stop all outputs.
    pub fn update(
        &mut self,
        button: bool,
        manual: bool,
        height: Angle,
        pivot: Angle,
        limits: LowScoreLimits,
        elapsed: Duration,
    ) -> Result<Option<LowScoreCommand>, LowScoreError> {
        if !limits.lift.valid()
            || !limits.pivot.valid()
            || ![height, pivot, limits.low_height]
                .iter()
                .all(|v| v.value.is_finite())
            || limits.eject_time.is_zero()
            || limits.travel_timeout.is_zero()
        {
            self.cancel();
            return Err(LowScoreError::InvalidFeedback);
        }
        let low_height = limits.lift.target(limits.low_height);
        if !button {
            self.pressed = false;
        }
        if manual {
            // Leave a new B press pending while manual controls have priority.
            // A started cycle stays latched until B is released.
            self.cancel();
            return Ok(None);
        }
        let edge = core::mem::take(&mut self.requested) || (button && !self.pressed);
        self.pressed = button;
        if edge {
            if self.cycle.is_some() {
                self.cancel();
                return Ok(None);
            }
            self.cycle = Some(Cycle {
                phase: Phase::Lower,
                elapsed: Duration::ZERO,
                original_height: limits.lift.target(height),
                low_height: if low_height < limits.lift.target(height) {
                    low_height
                } else {
                    limits.lift.target(height)
                },
                pivot_angle: limits.pivot.target(pivot),
            });
        }
        let Some(cycle) = self.cycle.as_mut() else {
            return Ok(None);
        };
        let low_height = cycle.low_height;
        cycle.elapsed = cycle.elapsed.saturating_add(elapsed);
        match cycle.phase {
            Phase::Lower if height <= low_height + limits.lift.arrival_tolerance => {
                cycle.phase = Phase::Eject;
                cycle.elapsed = Duration::ZERO;
            }
            Phase::Eject if cycle.elapsed >= limits.eject_time => {
                cycle.phase = Phase::Return;
                cycle.elapsed = Duration::ZERO;
            }
            Phase::Return
                if (height - cycle.original_height).value.abs()
                    <= limits.lift.arrival_tolerance.value
                    && (pivot - cycle.pivot_angle).value.abs()
                        <= limits.pivot.arrival_tolerance.value =>
            {
                let command = LowScoreCommand {
                    lift_target: cycle.original_height,
                    hold_lift: false,
                    pivot_target: cycle.pivot_angle,
                    intake_direction: 0,
                };
                self.cancel();
                return Ok(Some(command));
            }
            _ => {}
        }
        if !matches!(cycle.phase, Phase::Eject) && cycle.elapsed >= limits.travel_timeout {
            self.cancel();
            return Err(LowScoreError::TravelTimeout);
        }
        Ok(Some(LowScoreCommand {
            hold_lift: matches!(cycle.phase, Phase::Eject),
            lift_target: if matches!(cycle.phase, Phase::Return) {
                cycle.original_height
            } else {
                low_height
            },
            pivot_target: cycle.pivot_angle,
            intake_direction: if matches!(cycle.phase, Phase::Eject) {
                -1
            } else {
                0
            },
        }))
    }
}

#[cfg(test)]
#[path = "low_score_tests.rs"]
mod tests;
