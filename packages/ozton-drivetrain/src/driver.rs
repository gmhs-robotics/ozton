//! Driver-only assistance shared by drivetrain models.
use std::time::Duration;

use ozton_control::{drive::AntiTip, heading_hold::HeadingHold};

use crate::model::ArcadeInput;

#[derive(Default)]
pub(super) struct DriverControl {
    pub(super) antitip: Option<AntiTip>,
    pub(super) headinghold: Option<HeadingHold>,
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

    pub(super) fn apply(&mut self, input: ArcadeInput, dt: Duration, strafe: f64) -> (f64, f64) {
        const MAX_GAP: Duration = Duration::from_millis(100);
        if dt > MAX_GAP {
            self.reset();
        }
        let forward = input.throttle;
        let turn = input.turn;
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
        (forward + correction, turn)
    }
}
