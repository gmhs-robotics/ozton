//! Driver-only IMU heading hold. No position estimation or distance sensors.

use vexide::math::Angle;

/// Holds the heading captured when straight driving starts. Manual steering,
/// stopping and invalid IMU feedback release the captured heading.
pub struct HeadingHold {
    target: Option<Angle>,
    gain: f64,
    maximum_turn: f64,
}

impl HeadingHold {
    pub fn new(gain: f64, maximum_turn: f64) -> Self {
        Self {
            target: None,
            gain,
            maximum_turn,
        }
    }

    pub fn reset(&mut self) {
        self.target = None;
    }

    /// Returns arcade turn input. Heading uses the Vexide API's clockwise-positive radians;
    /// positive arcade turn makes the left side faster and the right side slower.
    pub fn turn(&mut self, throttle: f64, manual_turn: f64, heading: Option<Angle>) -> f64 {
        if !throttle.is_finite()
            || !manual_turn.is_finite()
            || !self.gain.is_finite()
            || self.gain < 0.0
            || !self.maximum_turn.is_finite()
            || !(0.0..=1.0).contains(&self.maximum_turn)
        {
            self.reset();
            return 0.0;
        }
        if manual_turn != 0.0 || throttle == 0.0 {
            self.reset();
            return manual_turn;
        }
        let Some(heading) = heading.filter(|angle| angle.as_radians().is_finite()) else {
            self.reset();
            return 0.0;
        };
        let target = *self.target.get_or_insert(heading);
        let error = (target - heading).wrapped_half().as_radians();
        // Keep both sides in the requested travel direction, even at low throttle.
        let limit = self.maximum_turn.min(throttle.abs());
        (self.gain * error).clamp(-limit, limit)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn angle(degrees: f64) -> Option<Angle> {
        Some(Angle::from_degrees(degrees))
    }
    fn hold() -> HeadingHold {
        HeadingHold::new(1.5, 0.2)
    }

    #[test]
    fn left_and_right_drift_correct_with_same_yaw_sign_in_forward_and_reverse() {
        for throttle in [0.6, -0.6] {
            let mut hold = hold();
            assert_eq!(hold.turn(throttle, 0.0, angle(0.0)), 0.0);
            assert!(hold.turn(throttle, 0.0, angle(-5.0)) > 0.0);
            assert!(hold.turn(throttle, 0.0, angle(5.0)) < 0.0);
            assert_eq!(hold.turn(throttle, 0.0, angle(0.0)), 0.0);
        }
    }

    #[test]
    fn manual_turn_stop_fault_and_reset_release_heading() {
        let mut hold = hold();
        hold.turn(0.6, 0.0, angle(0.0));
        assert_eq!(hold.turn(0.6, 0.3, angle(20.0)), 0.3);
        assert_eq!(hold.turn(0.6, 0.0, angle(30.0)), 0.0);
        assert_eq!(hold.turn(0.0, 0.0, angle(40.0)), 0.0);
        assert_eq!(hold.turn(0.6, 0.0, angle(50.0)), 0.0);
        assert_eq!(hold.turn(0.6, 0.0, None), 0.0);
        assert_eq!(hold.turn(0.6, 0.0, angle(60.0)), 0.0);
        assert_eq!(
            hold.turn(0.6, 0.0, Some(Angle::from_radians(f64::NAN))),
            0.0
        );
        assert_eq!(hold.turn(0.6, 0.0, angle(70.0)), 0.0);
        hold.reset();
        assert_eq!(hold.turn(0.6, 0.0, angle(80.0)), 0.0);
    }

    #[test]
    fn correction_wraps_and_stays_bounded_at_low_and_full_throttle() {
        let mut hold = hold();
        hold.turn(1.0, 0.0, angle(359.0));
        let turn = hold.turn(1.0, 0.0, angle(1.0));
        assert!(turn < 0.0 && turn.abs() < 0.06);
        assert_eq!(hold.turn(1.0, 0.0, angle(90.0)), -0.2);
        assert_eq!(hold.turn(0.05, 0.0, angle(90.0)), -0.05);
    }
}
