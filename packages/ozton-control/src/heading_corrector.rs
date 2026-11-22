//! Heading hold for straight arcade commands.

use ozton_math::{geometry::wrap_heading, uom::si::f64::Angle};

/// Holds the heading captured when straight driving starts. Manual steering,
/// stopping and invalid IMU feedback release the captured heading.
pub struct HeadingCorrector {
    target: Option<Angle>,
    gain: f64,
    maximum_turn: f64,
}

impl HeadingCorrector {
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

    /// Returns arcade turn input. Heading uses clockwise-positive rotation;
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
        let Some(heading) = heading.filter(|angle| angle.value.is_finite()) else {
            self.reset();
            return 0.0;
        };
        let target = *self.target.get_or_insert(heading);
        let error = wrap_heading(target - heading);
        // Keep both sides in the requested travel direction, even at low throttle.
        let limit = self.maximum_turn.min(throttle.abs());
        (self.gain * error).clamp(-limit, limit)
    }
}

#[cfg(test)]
mod tests {
    use ozton_math::uom::si::angle::{degree, radian};

    use super::*;
    fn angle(degrees: f64) -> Option<Angle> {
        Some(Angle::new::<degree>(degrees))
    }
    fn hold() -> HeadingCorrector {
        HeadingCorrector::new(1.5, 0.2)
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
            hold.turn(0.6, 0.0, Some(Angle::new::<radian>(f64::NAN))),
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
