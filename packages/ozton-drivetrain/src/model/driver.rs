//! Driver-only assistance and output limits shared by drivetrain models.
use std::time::Duration;

use ozton_control::{drive::AntiTip, heading_hold::HeadingHold};

use super::ArcadeInput;

pub(super) struct DriverControl {
    pub(super) antitip: Option<AntiTip>,
    pub(super) headinghold: Option<HeadingHold>,
    pub(super) slowdown: Option<(f64, f64)>,
    pub(super) scale: f64,
    pub(super) deadband: f64,
}

impl Default for DriverControl {
    fn default() -> Self {
        Self {
            antitip: None,
            headinghold: None,
            slowdown: None,
            scale: 1.0,
            deadband: 0.0,
        }
    }
}

impl DriverControl {
    pub(super) fn reset(&mut self) {
        if let Some(hold) = &mut self.headinghold {
            hold.reset();
        }
        if let Some(tip) = &mut self.antitip {
            tip.reset();
        }
    }

    pub(super) fn axis(&self, value: f64) -> f64 {
        if value.is_finite() && value.abs() > self.deadband {
            value
        } else {
            0.0
        }
    }

    pub(super) fn apply(
        &mut self,
        input: ArcadeInput,
        dt: Duration,
        strafe: f64,
    ) -> (f64, f64, f64) {
        const MAX_GAP: Duration = Duration::from_millis(100);
        if dt > MAX_GAP {
            self.reset();
        }
        let forward = self.axis(input.throttle);
        let turn = self.axis(input.turn);
        let turn = self.headinghold.as_mut().map_or(turn, |hold| {
            hold.turn(forward.abs().max(strafe.abs()), turn, input.heading)
        });
        let correction = self.antitip.as_mut().map_or(0.0, |tip| {
            if let Some(tilt) = input.tilt.filter(|tilt| tilt.value.is_finite()) {
                tip.correction(tilt, dt.min(MAX_GAP))
            } else {
                tip.reset();
                0.0
            }
        });
        let cap = self.slowdown.map_or(1.0, |(full_load, minimum)| {
            let fraction = input
                .load
                .filter(|load| load.is_finite())
                .map_or(1.0, |load| (load / full_load).clamp(0.0, 1.0));
            1.0 - (1.0 - minimum) * fraction
        });
        (
            forward + correction,
            turn,
            if input.boost { cap } else { self.scale * cap },
        )
    }
}

macro_rules! driver_builders {
    () => {
        pub fn with_antitip(mut self, controller: ozton_control::drive::AntiTip) -> Self {
            self.driver.antitip = Some(controller);
            self
        }

        pub fn with_headinghold(self) -> Self {
            self.with_headinghold_tuning(1.5, 0.20)
        }

        pub fn with_headinghold_tuning(mut self, gain: f64, maximum_turn: f64) -> Self {
            self.driver.headinghold = Some(ozton_control::heading_hold::HeadingHold::new(
                gain,
                maximum_turn,
            ));
            self
        }

        pub fn with_driver_scale(mut self, scale: f64) -> Self {
            self.driver.scale = if scale.is_finite() {
                scale.clamp(0.0, 1.0)
            } else {
                0.0
            };
            self
        }

        pub fn with_deadband(mut self, deadband: f64) -> Self {
            assert!((0.0..1.0).contains(&deadband), "invalid driver deadband");
            self.driver.deadband = deadband;
            self
        }

        /// Scale manual drive linearly with load. Missing feedback uses the minimum scale.
        pub fn with_slowdown(mut self, full_load: f64, minimum_scale: f64) -> Self {
            assert!(
                full_load.is_finite() && full_load > 0.0 && (0.0..=1.0).contains(&minimum_scale),
                "invalid driver slowdown"
            );
            self.driver.slowdown = Some((full_load, minimum_scale));
            self
        }
    };
}
pub(super) use driver_builders;

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn load_cap_applies_after_boost_and_fails_conservative() {
        let mut control = DriverControl {
            slowdown: Some((700.0, 0.5)),
            scale: 0.75,
            ..Default::default()
        };
        for (load, expected) in [
            (Some(0.0), 1.0),
            (Some(350.0), 0.75),
            (None, 0.5),
            (Some(f64::NAN), 0.5),
        ] {
            let input = ArcadeInput::new(1.0, 0.0).with_load(load);
            assert_eq!(control.apply(input, Duration::ZERO, 0.0).2, 0.75 * expected);
            assert_eq!(
                control.apply(input.with_boost(true), Duration::ZERO, 0.0).2,
                expected
            );
        }
    }
}
