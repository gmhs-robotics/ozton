//! Common-command control for a mirrored two-motor lift.

use std::time::Duration;

use ozton_math::uom::si::{angle::degree, f64::Angle};

/// Voltage limits and recorded-position control for a mirrored lift.
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

/// Produces one command shared by both lift motors.
pub struct LiftController {
    profile: LiftProfile,
    command: f64,
}

impl LiftController {
    pub const fn new(profile: LiftProfile) -> Self {
        Self {
            profile,
            command: 0.0,
        }
    }

    /// Forget ramp state after an external stop or hold.
    pub fn reset(&mut self) {
        self.command = 0.0;
    }

    /// Live control follows the button directly. Playback follows the recorded position.
    pub fn update(
        &mut self,
        requested_direction: i8,
        target: Option<Angle>,
        position: Option<Angle>,
        dt: Duration,
    ) -> f64 {
        let target = target.map(|angle| angle.get::<degree>());
        let position = position.map(|angle| angle.get::<degree>());
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
            None => f64::from(requested_direction.signum()),
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

    use super::{Angle, LiftController, LiftProfile, degree};

    fn controller() -> LiftController {
        LiftController::new(LiftProfile {
            bottom: Angle::new::<degree>(30.0),
            top: Angle::new::<degree>(600.0),
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
            (lift.update(1, None, Some(Angle::new::<degree>(100.0)), step) - 0.15).abs() < 1e-12
        );
        assert!(
            (lift.update(1, None, Some(Angle::new::<degree>(100.0)), step) - 0.30).abs() < 1e-12
        );
        assert_eq!(
            lift.update(
                1,
                None,
                Some(Angle::new::<degree>(100.0)),
                Duration::from_secs(1)
            ),
            0.75
        );
        assert_eq!(
            lift.update(0, None, Some(Angle::new::<degree>(100.0)), step),
            0.0
        );
        assert!(
            (lift.update(-1, None, Some(Angle::new::<degree>(100.0)), step) + 0.09).abs() < 1e-12
        );
        assert_eq!(
            lift.update(
                -1,
                None,
                Some(Angle::new::<degree>(100.0)),
                Duration::from_secs(1)
            ),
            -0.33
        );
        assert!(
            (lift.update(1, None, Some(Angle::new::<degree>(100.0)), step) - 0.15).abs() < 1e-12
        );
    }

    #[test]
    fn known_endpoints_stop_motion_outward() {
        let mut lift = controller();
        let step = Duration::from_millis(100);
        assert_eq!(
            lift.update(1, None, Some(Angle::new::<degree>(600.0)), step),
            0.0
        );
        assert_eq!(
            lift.update(-1, None, Some(Angle::new::<degree>(30.0)), step),
            0.0
        );
        assert!(
            (lift.update(1, None, Some(Angle::new::<degree>(100.0)), step) - 0.15).abs() < 1e-12
        );
        assert_eq!(
            lift.update(1, None, Some(Angle::new::<degree>(600.0)), step),
            0.0
        );
        assert!(
            (lift.update(-1, None, Some(Angle::new::<degree>(100.0)), step) + 0.09).abs() < 1e-12
        );
    }

    #[test]
    fn playback_respects_position_command_and_ramp() {
        let mut lift = controller();
        let step = Duration::from_millis(100);
        assert!(
            (lift.update(
                1,
                Some(Angle::new::<degree>(120.0)),
                Some(Angle::new::<degree>(100.0)),
                step
            ) - 0.15)
                .abs()
                < 1e-12
        );
        assert_eq!(
            lift.update(
                1,
                Some(Angle::new::<degree>(102.0)),
                Some(Angle::new::<degree>(100.0)),
                step
            ),
            0.0
        );
        assert!(
            (lift.update(
                -1,
                Some(Angle::new::<degree>(80.0)),
                Some(Angle::new::<degree>(100.0)),
                step
            ) + 0.09)
                .abs()
                < 1e-12
        );
    }
}
