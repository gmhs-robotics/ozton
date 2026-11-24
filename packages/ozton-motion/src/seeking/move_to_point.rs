use std::{
    future::Future,
    pin::Pin,
    task::Poll,
    time::{Duration, Instant},
};

use glam::DVec2 as Vec2;
use ozton_control::{
    Tolerances,
    axis::Linear,
    loops::{Feedback, Pid},
};
use ozton_drivetrain::{Drive, HasLocalization};
use ozton_math::{
    geometry::{bearing, wrap_heading},
    localization::observation::{TracksHeading, TracksPosition, TracksVelocity},
    uom::si::{
        f64::{Length, Velocity},
        velocity::meter_per_second,
    },
};
use ozton_runtime::clock::{Sleep, sleep};

use crate::Arcade;

pub(crate) struct State {
    sleep: Sleep,
    settling: ozton_control::Settling,
    prev_time: Instant,
    start_time: Instant,
}

/// Moves the robot to a point using two seeking feedback controllers.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct MoveToPointFuture<'a, M, L, A>
where
    M: Drive,
    L: Feedback<State = f64, Signal = f64> + Unpin,
    A: Feedback<State = f64, Signal = f64> + Unpin,
    M: HasLocalization<Localization: TracksPosition + TracksHeading + TracksVelocity>,
{
    pub(crate) target_point: Vec2,
    pub(crate) reverse: bool,
    pub(crate) timeout: Option<Duration>,
    pub(crate) tolerances: Tolerances<Linear>,
    pub(crate) linear_controller: L,
    pub(crate) lateral_controller: A,
    pub(crate) drivetrain: &'a mut M,
    pub(crate) state: Option<State>,
}

// MARK: Future Poll

impl<M, L, A> Drop for MoveToPointFuture<'_, M, L, A>
where
    M: Drive,
    L: Feedback<State = f64, Signal = f64> + Unpin,
    A: Feedback<State = f64, Signal = f64> + Unpin,
    M: HasLocalization<Localization: TracksPosition + TracksHeading + TracksVelocity>,
{
    fn drop(&mut self) {
        if self.state.is_some() {
            let _ = self.drivetrain.stop_now();
        }
    }
}

impl<M, L, A> Future for MoveToPointFuture<'_, M, L, A>
where
    M: Drive,
    L: Feedback<State = f64, Signal = f64> + Unpin,
    A: Feedback<State = f64, Signal = f64> + Unpin,
    M: HasLocalization<Localization: TracksPosition + TracksHeading + TracksVelocity>,
{
    type Output = Result<crate::MotionOutcome, crate::MotionError<M::Error>>;

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> Poll<Self::Output> {
        let this = self.get_mut();
        let state = this.state.get_or_insert_with(|| {
            let now = Instant::now();

            State {
                sleep: sleep(Duration::from_millis(10)),
                start_time: now,
                prev_time: now,
                settling: Default::default(),
            }
        });

        if Pin::new(&mut state.sleep).poll(cx).is_pending() {
            return Poll::Pending;
        }

        let now = Instant::now();
        let dt = now.saturating_duration_since(state.prev_time);

        let position = this.drivetrain.localization().position();
        let heading = this.drivetrain.localization().heading();
        let velocity = this.drivetrain.localization().linear_velocity();
        let local_target = this.target_point - position;
        let distance_error = local_target.length();
        let feedback_error = if !this.target_point.is_finite() {
            Some(crate::MotionError::InvalidTarget)
        } else if !distance_error.is_finite() || !heading.value.is_finite() || !velocity.is_finite()
        {
            Some(crate::MotionError::InvalidFeedback)
        } else {
            None
        };
        if let Some(error) = feedback_error {
            this.state = None;
            return Poll::Ready(stop_with_error(this.drivetrain, error));
        }

        let settled = this.tolerances.check_with(
            &mut state.settling,
            ozton_math::length!(distance_error, m),
            Velocity::new::<meter_per_second>(velocity),
            now,
        );
        let timed_out = this
            .timeout
            .is_some_and(|timeout| now.saturating_duration_since(state.start_time) >= timeout);
        if settled || timed_out {
            this.state = None;
            let outcome = if settled {
                crate::MotionOutcome::Settled
            } else {
                crate::MotionOutcome::TimedOut
            };
            return Poll::Ready(crate::finish_motion(this.drivetrain.stop_now(), outcome));
        }

        let within_position_tolerance = distance_error == 0.0
            || this
                .tolerances
                .error_tolerance
                .is_some_and(|tolerance| distance_error < tolerance.value);
        let (linear_output, angular_output) = if within_position_tolerance {
            (0.0, 0.0)
        } else {
            let drive_heading = if this.reverse {
                heading + ozton_math::angle!(std::f64::consts::PI, rad)
            } else {
                heading
            };
            let angle_error = wrap_heading(drive_heading - bearing(local_target));
            // Keep steering authority close to the goal and when facing away from it.
            let steering_error = angle_error.value * distance_error.max(0.5);
            let angular = this.lateral_controller.update(-steering_error, 0.0, dt);
            let direction = if this.reverse { -1.0 } else { 1.0 };
            let linear = this
                .linear_controller
                .update(-direction * distance_error, 0.0, dt);
            let alignment = (2.0 * angle_error.value.cos() - 1.0).max(0.0);
            (linear * alignment * alignment, angular)
        };

        if !linear_output.is_finite() || !angular_output.is_finite() {
            this.state = None;
            return Poll::Ready(stop_with_error(
                this.drivetrain,
                crate::MotionError::InvalidOutput,
            ));
        }

        if let Err(output) = Arcade::drive(this.drivetrain, linear_output, angular_output) {
            this.state = None;
            return Poll::Ready(Err(crate::motion_output_error(
                output,
                this.drivetrain.stop_now(),
            )));
        }

        state.sleep = sleep(Duration::from_millis(10));
        state.prev_time = now;

        cx.waker().wake_by_ref();
        Poll::Pending
    }
}

fn stop_with_error<M: Drive>(
    drivetrain: &mut M,
    error: crate::MotionError<M::Error>,
) -> Result<crate::MotionOutcome, crate::MotionError<M::Error>> {
    drivetrain.stop_now().map_err(crate::MotionError::Stop)?;
    Err(error)
}

// MARK: Generic Modifiers

impl<M, L, A> MoveToPointFuture<'_, M, L, A>
where
    M: Drive,
    L: Feedback<State = f64, Signal = f64> + Unpin,
    A: Feedback<State = f64, Signal = f64> + Unpin,
    M: HasLocalization<Localization: TracksPosition + TracksHeading + TracksVelocity>,
{
    /// Reverses this motion, moving to the point backwards rather than forwards.
    pub fn reverse(&mut self) -> &mut Self {
        self.reverse = true;
        self
    }

    /// Modifies this motion's linear feedback controller.
    pub fn with_linear_controller(&mut self, controller: L) -> &mut Self {
        self.linear_controller = controller;
        self
    }

    /// Modifies this motion's lateral feedback controller.
    pub fn with_lateral_controller(&mut self, controller: A) -> &mut Self {
        self.lateral_controller = controller;
        self
    }

    /// Modifies this motion's timeout duration.
    pub const fn with_timeout(&mut self, timeout: Duration) -> &mut Self {
        self.timeout = Some(timeout);
        self
    }

    /// Removes this motion's timeout duration.
    pub const fn without_timeout(&mut self) -> &mut Self {
        self.timeout = None;
        self
    }

    /// Modifies this motion's tolerances.
    pub const fn with_tolerances(&mut self, tolerances: Tolerances<Linear>) -> &mut Self {
        self.tolerances = tolerances;
        self
    }

    /// Modifies this motion's error tolerance.
    pub const fn with_error_tolerance(&mut self, tolerance: Length) -> &mut Self {
        self.tolerances.error_tolerance = Some(tolerance);
        self
    }

    /// Removes this motion's error tolerance.
    pub const fn without_error_tolerance(&mut self) -> &mut Self {
        self.tolerances.error_tolerance = None;
        self
    }

    /// Modifies this motion's velocity tolerance.
    pub const fn with_velocity_tolerance(&mut self, tolerance: Velocity) -> &mut Self {
        self.tolerances.velocity_tolerance = Some(tolerance);
        self
    }

    /// Removes this motion's velocity tolerance.
    pub const fn without_velocity_tolerance(&mut self) -> &mut Self {
        self.tolerances.velocity_tolerance = None;
        self
    }

    /// Modifies this motion's tolerance duration.
    pub const fn with_tolerance_duration(&mut self, duration: Duration) -> &mut Self {
        self.tolerances.duration = Some(duration);
        self
    }

    /// Removes this motion's tolerance duration.
    pub const fn without_tolerance_duration(&mut self) -> &mut Self {
        self.tolerances.duration = None;
        self
    }
}

// MARK: Linear PID Modifiers

impl<M, A> MoveToPointFuture<'_, M, Pid, A>
where
    M: Drive,
    A: Feedback<State = f64, Signal = f64> + Unpin,
    M: HasLocalization<Localization: TracksPosition + TracksHeading + TracksVelocity>,
{
    /// Modifies this motion's linear PID gains.
    pub const fn with_linear_gains(&mut self, kp: f64, ki: f64, kd: f64) -> &mut Self {
        self.linear_controller.set_gains(kp, ki, kd);
        self
    }

    /// Modifies this motion's linear proportional gain (`kp`).
    pub const fn with_linear_kp(&mut self, kp: f64) -> &mut Self {
        self.linear_controller.set_kp(kp);
        self
    }

    /// Modifies this motion's linear integral gain (`ki`).
    pub const fn with_linear_ki(&mut self, ki: f64) -> &mut Self {
        self.linear_controller.set_ki(ki);
        self
    }

    /// Modifies this motion's linear derivative gain (`kd`).
    pub const fn with_linear_kd(&mut self, kd: f64) -> &mut Self {
        self.linear_controller.set_kd(kd);
        self
    }

    /// Modifies this motion's linear integration range.
    pub const fn with_linear_integration_range(&mut self, integration_range: Length) -> &mut Self {
        self.linear_controller
            .set_integration_range(Some(integration_range.value));
        self
    }

    /// Removes this motion's linear integration range.
    pub const fn without_linear_integration_range(&mut self) -> &mut Self {
        self.linear_controller.set_integration_range(None);
        self
    }

    /// Modifies this motion's linear output limit as a fraction of motor voltage.
    pub const fn with_linear_output_limit(&mut self, limit: f64) -> &mut Self {
        self.linear_controller.set_output_limit(Some(limit));
        self
    }

    /// Sets the linear output limit in volts for V5 motors.
    pub const fn with_linear_voltage_limit(&mut self, volts: f64) -> &mut Self {
        self.with_linear_output_limit(volts / ozton_drivetrain::model::Differential::MAX_V5_VOLTAGE)
    }

    /// Removes this motion's linear output limit.
    pub const fn without_linear_output_limit(&mut self) -> &mut Self {
        self.linear_controller.set_output_limit(None);
        self
    }
}

// MARK: Angular PID Modifiers

impl<M, L> MoveToPointFuture<'_, M, L, Pid>
where
    M: Drive,
    L: Feedback<State = f64, Signal = f64> + Unpin,
    M: HasLocalization<Localization: TracksPosition + TracksHeading + TracksVelocity>,
{
    /// Modifies this motion's angular PID gains.
    pub const fn with_angular_gains(&mut self, kp: f64, ki: f64, kd: f64) -> &mut Self {
        self.with_lateral_gains(kp, ki, kd)
    }

    /// Modifies this motion's angular proportional gain (`kp`).
    pub const fn with_angular_kp(&mut self, kp: f64) -> &mut Self {
        self.with_lateral_kp(kp)
    }

    /// Modifies this motion's lateral PID gains.
    pub const fn with_lateral_gains(&mut self, kp: f64, ki: f64, kd: f64) -> &mut Self {
        self.lateral_controller.set_gains(kp, ki, kd);
        self
    }

    /// Modifies this motion's lateral proportional gain (`kp`).
    pub const fn with_lateral_kp(&mut self, kp: f64) -> &mut Self {
        self.lateral_controller.set_kp(kp);
        self
    }

    /// Modifies this motion's lateral integral gain (`ki`).
    pub const fn with_lateral_ki(&mut self, ki: f64) -> &mut Self {
        self.lateral_controller.set_ki(ki);
        self
    }

    /// Modifies this motion's lateral derivative gain (`kd`).
    pub const fn with_lateral_kd(&mut self, kd: f64) -> &mut Self {
        self.lateral_controller.set_kd(kd);
        self
    }

    /// Modifies this motion's lateral integration range.
    pub const fn with_lateral_integration_range(&mut self, integration_range: Length) -> &mut Self {
        self.lateral_controller
            .set_integration_range(Some(integration_range.value));
        self
    }

    /// Modifies this motion's lateral output limit.
    pub const fn with_lateral_output_limit(&mut self, limit: f64) -> &mut Self {
        self.lateral_controller.set_output_limit(Some(limit));
        self
    }

    /// Removes this motion's lateral integration range.
    pub const fn without_lateral_integration_range(&mut self) -> &mut Self {
        self.lateral_controller.set_integration_range(None);
        self
    }

    /// Removes this motion's lateral output limit.
    pub const fn without_lateral_output_limit(&mut self) -> &mut Self {
        self.lateral_controller.set_output_limit(None);
        self
    }
}
