//! PID configuration for wheel-velocity autonomous commands.

use ozton_math::control::Pid;

/// Gains for the outer motion loop. Linear feedback uses metres and commands
/// metres per second. Angular feedback uses radians and commands radians per second.
/// PID output limits apply to the feedback term of each loop.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DriveTuning {
    pub linear: Pid,
    pub angular: Pid,
}

impl DriveTuning {
    pub const STANDARD: Self = Self {
        linear: Pid::new(3.1, 0.6, 0.12, Some(0.08)),
        angular: Pid::new(2.2, 0.3, 0.12, Some(0.24)),
    };

    pub(super) fn valid(self) -> bool {
        [self.linear, self.angular].into_iter().all(|pid| {
            let (kp, ki, kd) = pid.gains();
            [kp, ki, kd].into_iter().all(f64::is_finite)
                && [pid.integration_range(), pid.output_limit()]
                    .into_iter()
                    .flatten()
                    .all(|value| value.is_finite() && value >= 0.0)
        })
    }
}

impl Default for DriveTuning {
    fn default() -> Self {
        Self::STANDARD
    }
}
