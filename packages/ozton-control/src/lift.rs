//! Common-command control for a mirrored two-motor lift.

use std::time::Duration;

use ozton_math::uom::si::{angle::degree, f64::Angle};

/// Voltage limits and position control for a mirrored lift.
#[derive(Debug, Clone, Copy)]
pub struct LiftProfile {
    pub bottom: Angle,
    pub top: Angle,
    pub up_maximum_command: f64,
    pub down_maximum_command: f64,
    /// Maximum increase in upward command magnitude per second.
    pub up_rise_per_second: f64,
    /// Maximum increase in downward command magnitude per second.
    pub down_rise_per_second: f64,
    pub position_gain: f64,
}

impl LiftProfile {
    pub const fn is_valid(self) -> bool {
        self.bottom.value.is_finite()
            && self.top.value.is_finite()
            && self.top.value > self.bottom.value
            && self.up_maximum_command >= 0.0
            && self.up_maximum_command <= 1.0
            && self.down_maximum_command >= 0.0
            && self.down_maximum_command <= 1.0
            && self.up_rise_per_second.is_finite()
            && self.up_rise_per_second >= 0.0
            && self.down_rise_per_second.is_finite()
            && self.down_rise_per_second >= 0.0
            && self.position_gain.is_finite()
            && self.position_gain >= 0.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LiftDirection {
    Up,
    Hold,
    Down,
}

impl LiftDirection {
    pub const fn from_signed(direction: i8) -> Self {
        match direction.signum() {
            1 => Self::Up,
            -1 => Self::Down,
            _ => Self::Hold,
        }
    }

    const fn sign(self) -> f64 {
        match self {
            Self::Up => 1.0,
            Self::Down => -1.0,
            Self::Hold => 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LiftTarget {
    Manual(LiftDirection),
    Position(Angle),
}

/// Produces one command shared by both lift motors.
pub struct LiftController {
    profile: LiftProfile,
    command: f64,
}

impl LiftController {
    pub const fn new(profile: LiftProfile) -> Self {
        assert!(profile.is_valid(), "invalid lift profile");
        Self {
            profile,
            command: 0.0,
        }
    }

    /// Forget ramp state after an external stop or hold.
    pub fn reset(&mut self) {
        self.command = 0.0;
    }

    /// Manual control uses a signed direction. Position control follows a target.
    pub fn update(
        &mut self,
        requested_direction: i8,
        target: Option<Angle>,
        position: Option<Angle>,
        dt: Duration,
    ) -> f64 {
        let target = target.map_or(
            LiftTarget::Manual(LiftDirection::from_signed(requested_direction)),
            LiftTarget::Position,
        );
        self.update_target(target, position, dt)
    }

    pub fn update_target(
        &mut self,
        target: LiftTarget,
        position: Option<Angle>,
        dt: Duration,
    ) -> f64 {
        let (manual_direction, target) = match target {
            LiftTarget::Manual(direction) => (direction.sign(), None),
            LiftTarget::Position(target) => (0.0, Some(target.get::<degree>())),
        };
        let position = position.map(|angle| angle.get::<degree>());
        if target.is_some_and(|target| !target.is_finite())
            || position.is_some_and(|position| !position.is_finite())
        {
            self.reset();
            return 0.0;
        }
        let direction = match target {
            Some(target) if position.is_some_and(|position| (target - position).abs() <= 3.0) => {
                self.reset();
                return 0.0;
            }
            Some(target) => match position {
                Some(position) => (target - position).signum(),
                None => {
                    self.reset();
                    return 0.0;
                }
            },
            None => manual_direction,
        };
        if direction == 0.0 {
            self.reset();
            return 0.0;
        }
        if position.is_some_and(|position| {
            direction > 0.0 && position >= self.profile.top.get::<degree>()
                || direction < 0.0 && position <= self.profile.bottom.get::<degree>()
        }) {
            self.reset();
            return 0.0;
        }
        let limit = if direction > 0.0 {
            self.profile.up_maximum_command
        } else {
            self.profile.down_maximum_command
        };
        let magnitude = match (target, position) {
            (Some(target), Some(position)) => {
                ((target - position).abs() * self.profile.position_gain).min(limit)
            }
            _ => limit,
        };
        if self.command.signum() != direction {
            self.reset();
        }
        let rise = if direction > 0.0 {
            self.profile.up_rise_per_second
        } else {
            self.profile.down_rise_per_second
        };
        self.command = direction * (self.command.abs() + rise * dt.as_secs_f64()).min(magnitude);
        self.command
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::{LiftController, LiftProfile};

    fn controller() -> LiftController {
        LiftController::new(LiftProfile {
            bottom: ozton_math::qty!(30.0 deg),
            top: ozton_math::qty!(600.0 deg),
            up_maximum_command: 0.75,
            down_maximum_command: 0.33,
            up_rise_per_second: 1.5,
            down_rise_per_second: 0.9,
            position_gain: 0.01,
        })
    }

    #[test]
    fn live_buttons_ramp_up_and_down_then_stop_immediately() {
        let mut lift = controller();
        let step = Duration::from_millis(100);
        assert!(
            (lift.update(1, None, Some(ozton_math::qty!(100.0 deg)), step) - 0.15).abs() < 1e-12
        );
        assert!(
            (lift.update(1, None, Some(ozton_math::qty!(100.0 deg)), step) - 0.30).abs() < 1e-12
        );
        assert_eq!(
            lift.update(
                1,
                None,
                Some(ozton_math::qty!(100.0 deg)),
                Duration::from_secs(1)
            ),
            0.75
        );
        assert_eq!(
            lift.update(0, None, Some(ozton_math::qty!(100.0 deg)), step),
            0.0
        );
        assert!(
            (lift.update(-1, None, Some(ozton_math::qty!(100.0 deg)), step) + 0.09).abs() < 1e-12
        );
        assert_eq!(
            lift.update(
                -1,
                None,
                Some(ozton_math::qty!(100.0 deg)),
                Duration::from_secs(1)
            ),
            -0.33
        );
        assert!(
            (lift.update(1, None, Some(ozton_math::qty!(100.0 deg)), step) - 0.15).abs() < 1e-12
        );
    }

    #[test]
    fn known_endpoints_stop_motion_outward() {
        let mut lift = controller();
        let step = Duration::from_millis(100);
        assert_eq!(
            lift.update(1, None, Some(ozton_math::qty!(600.0 deg)), step),
            0.0
        );
        assert_eq!(
            lift.update(-1, None, Some(ozton_math::qty!(30.0 deg)), step),
            0.0
        );
        assert!(
            (lift.update(1, None, Some(ozton_math::qty!(100.0 deg)), step) - 0.15).abs() < 1e-12
        );
        assert_eq!(
            lift.update(1, None, Some(ozton_math::qty!(600.0 deg)), step),
            0.0
        );
        assert!(
            (lift.update(-1, None, Some(ozton_math::qty!(100.0 deg)), step) + 0.09).abs() < 1e-12
        );
    }

    #[test]
    fn position_control_respects_command_and_ramp() {
        let mut lift = controller();
        let step = Duration::from_millis(100);
        assert!(
            (lift.update(
                1,
                Some(ozton_math::qty!(120.0 deg)),
                Some(ozton_math::qty!(100.0 deg)),
                step
            ) - 0.15)
                .abs()
                < 1e-12
        );
        assert_eq!(
            lift.update(
                1,
                Some(ozton_math::qty!(102.0 deg)),
                Some(ozton_math::qty!(100.0 deg)),
                step
            ),
            0.0
        );
        assert!(
            (lift.update(
                -1,
                Some(ozton_math::qty!(80.0 deg)),
                Some(ozton_math::qty!(100.0 deg)),
                step
            ) + 0.09)
                .abs()
                < 1e-12
        );
    }
}
