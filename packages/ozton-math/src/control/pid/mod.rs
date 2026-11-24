use core::time::Duration;

use super::Feedback;
use crate::geometry::wrap_heading;

mod scalar;
use scalar::ScalarPid;

/// PID controller.
///
/// The PID controller is one of the most widely used feedback control algorithms in industrial
/// control systems, robotics, and process control. It computes a control signal based on three
/// terms: **proportional**, **integral**, and **derivative**.
///
/// # Components
///
/// - The **proportional** term produces an output proportional to the current error, multiplied by
///   the gain constant `kp`. This provides the main control action: larger errors result in larger
///   corrective responses and vice versa.
///
/// - The **integral** term sums the error over time, multiplied by the gain constant `ki`. This
///   term eliminates *steady-state error* by ensuring that even small errors will eventually
///   accumulate a large enough response to reach the setpoint. Without integral action, a system
///   might stabilize with a small, persistent error if the control signal becomes too small to
///   overcome system friction, gravity, or other external factors on the system. For example, a
///   motor might need some minimum voltage to start moving, or a drone might need extra thrust to
///   hover against gravity. The integral term accumulates over time to provide this additional
///   correction.
///
/// - The **derivative** term measures the error's change over time, multiplied by the gain constant
///   `kd`. This provides a damping effect that reduces overshoot and oscillation by counteracting
///   rapid changes in error. The derivative term helps anticipate and smooth out the system's
///   response, preventing sudden changes resulting from large proportional or integral gains.
///
/// # Tuning
///
/// Tuning a PID controller requires adjusting `kp`, `ki`, and `kd` to allow the system to reach a
/// setpoint in a reasonable amount of time without oscillations (rapid, unpredictable changes in
/// output).
///
/// Tuning methods are typically dependent on the application that the PID controller is used in,
/// but a common method is as follows:
///
/// 1. Start with all gains at zero (`kp = 0.0`, `ki = 0.0`, `kd = 0.0`).
///
/// 2. Tune proportional gain first:
///    - Gradually increase `kp` until the system starts to oscillate around the setpoint.
///    - *Oscillation* occurs when the system reaches and overshoots the setpoint, then repeatedly
///      overadjusts itself around the setpoint, resulting in a "back-and-fourth" motion around the
///      setpoint.
///
/// 3. Tune the derivative gain:
///    - Start with a very small `kd` gain (0.05 × `kp` or less is a safe bet to start with).
///    - Gradually increase by small increments until oscillations from the proportional term stop
///      occurring.
///
/// 4. Add integral gain if necessary:
///    - Integral gain is only necessary if your controller's proportional and derivative terms
///      become small enough to where they can no longer overcome some external factor (such as
///      friction) of the system, resulting in what's called *steady-state error*.
///    - Start with a very small `ki` gain (such as 0.01 × `kp`).
///    - Increase `ki` slowly until steady-state errors are eliminated within an acceptable time.
///    - If oscillation occurs, reduce both `ki` and `kp` slightly.
///
/// Common signs of poor tuning:
///
/// - Slow response: `kp` is too low.
/// - Excessive overshoot: `kd` is too low or `ki` is too high.
/// - Oscillation: `kp` is too high or `kd` is too low.
/// - Noisy, unpredictable response: `kd` is too high.
///
/// # Integral Windup (and Mitigations)
///
/// In some scenarios, a PID controller may be prone to *integral windup*, where a controlled system
/// reaches a saturation point preventing the error from decreasing. In this case, integral will
/// rapidly accumulate, causing large and unpredictable control signals. This specific
/// implementation provides two mitigations for integral windup:
///
/// 1. **Sign-based reset:** When the sign of error changes (in other words, when the controller has
///    crossed/overshot its target), the integral value is reset to prevent overshoot of the target.
/// 2. **Integration bounds:** An optional `integration_range` value can be passed to the
///    controller, which defines a range of error where integration will occur. When
///    `|error| > integration_range`, no integration will occur if used.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pid {
    controller: ScalarPid,
    integration_range: Option<f64>,
}

impl Pid {
    pub const fn reset(&mut self) {
        self.controller.reset();
    }

    /// Adds feedforward and filters derivative-on-measurement. Output limits apply
    /// to the combined effort, including the conditional integral update.
    pub fn update_filtered(
        &mut self,
        measurement: f64,
        setpoint: f64,
        dt: Duration,
        filter: Duration,
        feedforward: f64,
    ) -> f64 {
        let error = setpoint - measurement;
        if dt.is_zero() || !error.is_finite() || !feedforward.is_finite() {
            return 0.0;
        }
        let derivative = self
            .controller
            .prev_measurement
            .map_or(0.0, |last| (last - measurement) / dt.as_secs_f64());
        self.controller.prev_measurement = Some(measurement);
        self.update_with_rate(error, derivative, dt, filter, feedforward)
    }

    /// PID with an externally measured error rate and optional feedforward.
    /// Useful for wrapped headings and localization estimates that can jump.
    pub fn update_with_rate(
        &mut self,
        error: f64,
        error_rate: f64,
        dt: Duration,
        filter: Duration,
        feedforward: f64,
    ) -> f64 {
        if dt.is_zero()
            || ![error, error_rate, feedforward]
                .iter()
                .all(|v| v.is_finite())
        {
            return 0.0;
        }
        let seconds = dt.as_secs_f64();
        self.controller.derivative += (error_rate - self.controller.derivative)
            * (seconds / (seconds + filter.as_secs_f64()));
        let integrate = self
            .integration_range
            .is_none_or(|range| error.abs() < range)
            && self.controller.integrate_at(error);
        self.controller.update(
            error,
            self.controller.derivative,
            seconds,
            integrate,
            feedforward,
        )
    }

    /// Construct a new PID controller from gain constants and an optional integration range.
    #[must_use]
    pub const fn new(kp: f64, ki: f64, kd: f64, integration_range: Option<f64>) -> Self {
        Self {
            controller: ScalarPid::new(kp, ki, kd),
            integration_range,
        }
    }

    /// Get the current PID gains as a tuple (`kp`, `ki`, `kd`).
    #[must_use]
    pub const fn gains(&self) -> (f64, f64, f64) {
        (self.controller.kp, self.controller.ki, self.controller.kd)
    }

    /// Returns the controller's proportional gain (`kp`).
    #[must_use]
    pub const fn kp(&self) -> f64 {
        self.controller.kp
    }

    /// Returns the controller's integral gain (`ki`).
    #[must_use]
    pub const fn ki(&self) -> f64 {
        self.controller.ki
    }

    /// Returns the controller's derivative gain (`kd`).
    #[must_use]
    pub const fn kd(&self) -> f64 {
        self.controller.kd
    }

    /// Returns the controller's integration range.
    ///
    /// Integration range is the minimum error range required to start integrating error. This is
    /// optionally applied to the controller as a mitigation for [integral windup].
    ///
    /// [integral windup]: https://en.wikipedia.org/wiki/Integral_windup
    #[must_use]
    pub const fn integration_range(&self) -> Option<f64> {
        self.integration_range
    }

    /// Returns the controller's output limit, or `None` if there is no
    /// limit applied.
    #[must_use]
    pub const fn output_limit(&self) -> Option<f64> {
        self.controller.output_limit
    }

    /// Sets the PID gains to provided values.
    pub const fn set_gains(&mut self, kp: f64, ki: f64, kd: f64) {
        self.controller.kp = kp;
        self.controller.ki = ki;
        self.controller.kd = kd;
    }

    /// Sets the controller's proportional gain (`kp`).
    pub const fn set_kp(&mut self, kp: f64) {
        self.controller.kp = kp;
    }

    /// Sets the controller's integral gain (`ki`).
    pub const fn set_ki(&mut self, ki: f64) {
        self.controller.ki = ki;
    }

    /// Sets the controller's derivative gain (`kd`).
    pub const fn set_kd(&mut self, kd: f64) {
        self.controller.kd = kd;
    }

    /// Sets the controller's integration range.
    ///
    /// Integration range is the minimum error range required to start integrating error. This is
    /// optionally applied to the controller as a mitigation for [integral windup].
    ///
    /// [integral windup]: https://en.wikipedia.org/wiki/Integral_windup
    pub const fn set_integration_range(&mut self, range: Option<f64>) {
        self.integration_range = range;
    }

    /// Sets the controller's output limit.
    ///
    /// This sets a maximum range for the controller's output signal. It will effectively limit how
    /// fast the controller is able to drive the system, which may be desirable in some cases (e.g.
    /// limiting the maximum speed of a robot's motion).
    pub const fn set_output_limit(&mut self, range: Option<f64>) {
        self.controller.output_limit = range;
    }

    #[must_use]
    pub const fn with_gains(mut self, kp: f64, ki: f64, kd: f64) -> Self {
        self.set_gains(kp, ki, kd);
        self
    }

    #[must_use]
    pub const fn with_kp(mut self, kp: f64) -> Self {
        self.set_kp(kp);
        self
    }

    #[must_use]
    pub const fn with_ki(mut self, ki: f64) -> Self {
        self.set_ki(ki);
        self
    }

    #[must_use]
    pub const fn with_kd(mut self, kd: f64) -> Self {
        self.set_kd(kd);
        self
    }

    #[must_use]
    pub const fn with_integration_range(mut self, range: f64) -> Self {
        self.set_integration_range(Some(range));
        self
    }

    #[must_use]
    pub const fn without_output_limit(mut self) -> Self {
        self.set_output_limit(None);
        self
    }

    #[must_use]
    pub const fn without_integration_range(mut self) -> Self {
        self.set_integration_range(None);
        self
    }

    /// Set the output limit while constructing a controller, including in constants.
    #[must_use]
    pub const fn with_output_limit(mut self, limit: f64) -> Self {
        self.set_output_limit(Some(limit));
        self
    }
}

impl Default for Pid {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0, None)
    }
}

impl Pid {
    /// PID on the shortest arc between two radian angles.
    pub fn update_angular(&mut self, measurement: f64, setpoint: f64, dt: Duration) -> f64 {
        let wrap = |error| wrap_heading(crate::qty!((error) rad)).value;
        self.update_error(wrap(setpoint - measurement), dt, wrap)
    }

    fn update_error(
        &mut self,
        error: f64,
        dt: Duration,
        derivative_error: impl FnOnce(f64) -> f64,
    ) -> f64 {
        if dt.is_zero() || !error.is_finite() {
            return 0.0;
        }
        let integrate = self
            .integration_range
            .is_none_or(|range| error.abs() < range)
            && self.controller.integrate_at(error);
        let seconds = dt.as_secs_f64();
        let derivative = self
            .controller
            .prev_error
            .map_or(0.0, |last| derivative_error(error - last) / seconds);
        self.controller
            .update(error, derivative, seconds, integrate, 0.0)
    }
}

impl Feedback for Pid {
    type State = f64;
    type Signal = f64;

    fn update(&mut self, measurement: f64, setpoint: f64, dt: Duration) -> f64 {
        self.update_error(setpoint - measurement, dt, |error| error)
    }
}
