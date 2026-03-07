use std::{
    f64::consts::FRAC_PI_2,
    future::Future,
    pin::Pin,
    task::Poll,
    time::{Duration, Instant},
};

use glam::DVec2 as Vec2;
use ozton_control::{
    Tolerances,
    loops::{Feedback, Pid},
};
use ozton_drivetrain::{Drivetrain, model::Arcade};
use ozton_tracking::{TracksHeading, TracksPosition, TracksVelocity};
use vexide::time::{Sleep, sleep};
use vexide_devices::math::Angle;

pub(crate) struct State {
    sleep: Sleep,
    prev_time: Instant,
    start_time: Instant,
}

/// Moves the robot to a point using two seeking feedback controllers.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct MoveToPointFuture<'a, M, L, A, T>
where
    M: Arcade,
    L: Feedback<State = f64, Signal = f64> + Unpin,
    A: Feedback<State = f64, Signal = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
{
    pub(crate) target_point: Vec2,
    pub(crate) reverse: bool,
    pub(crate) timeout: Option<Duration>,
    pub(crate) tolerances: Tolerances,
    pub(crate) linear_controller: L,
    pub(crate) lateral_controller: A,
    pub(crate) drivetrain: &'a mut Drivetrain<M, T>,
    pub(crate) state: Option<State>,
}

// MARK: Future Poll

impl<M, L, A, T> Future for MoveToPointFuture<'_, M, L, A, T>
where
    M: Arcade,
    L: Feedback<State = f64, Signal = f64> + Unpin,
    A: Feedback<State = f64, Signal = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
{
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> Poll<Self::Output> {
        let this = self.get_mut();
        let state = this.state.get_or_insert_with(|| {
            let now = Instant::now();

            State {
                sleep: sleep(Duration::from_millis(5)),
                start_time: now,
                prev_time: now,
            }
        });

        if Pin::new(&mut state.sleep).poll(cx).is_pending() {
            return Poll::Pending;
        }

        let dt = state.prev_time.elapsed();

        let position = this.drivetrain.tracking.position();
        let heading = this.drivetrain.tracking.heading();

        let local_target = this.target_point - position;
        let mut distance_error = local_target.length();

        if this
            .tolerances
            .check(distance_error, this.drivetrain.tracking.linear_velocity())
            || this
                .timeout
                .is_some_and(|timeout| state.start_time.elapsed() > timeout)
        {
            drop(this.drivetrain.model.drive_arcade(0.0, 0.0));
            return Poll::Ready(());
        }

        let angle_error =
            (heading - Angle::from_radians(local_target.y.atan2(local_target.x))).wrapped_half();
        let mut projected_cte = distance_error * angle_error.sin();

        if angle_error.as_radians().abs() > FRAC_PI_2 {
            projected_cte *= -1.0;
            distance_error *= -1.0;
        }

        let angular_output = this.lateral_controller.update(projected_cte, 0.0, dt);
        let linear_output =
            this.linear_controller.update(-distance_error, 0.0, dt) * angle_error.cos().abs();

        drop(
            this.drivetrain
                .model
                .drive_arcade(linear_output, angular_output),
        );

        state.sleep = sleep(Duration::from_millis(5));
        state.prev_time = Instant::now();

        cx.waker().wake_by_ref();
        Poll::Pending
    }
}

// MARK: Generic Modifiers

impl<M, L, A, T> MoveToPointFuture<'_, M, L, A, T>
where
    M: Arcade,
    L: Feedback<State = f64, Signal = f64> + Unpin,
    A: Feedback<State = f64, Signal = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
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
    pub const fn with_tolerances(&mut self, tolerances: Tolerances) -> &mut Self {
        self.tolerances = tolerances;
        self
    }

    /// Modifies this motion's error tolerance.
    pub const fn with_error_tolerance(&mut self, tolerance: f64) -> &mut Self {
        self.tolerances.error_tolerance = Some(tolerance);
        self
    }

    /// Removes this motion's error tolerance.
    pub const fn without_error_tolerance(&mut self) -> &mut Self {
        self.tolerances.error_tolerance = None;
        self
    }

    /// Modifies this motion's velocity tolerance.
    pub const fn with_velocity_tolerance(&mut self, tolerance: f64) -> &mut Self {
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

impl<M, A, T> MoveToPointFuture<'_, M, Pid, A, T>
where
    M: Arcade,
    A: Feedback<State = f64, Signal = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
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
    pub const fn with_linear_integration_range(&mut self, integration_range: f64) -> &mut Self {
        self.linear_controller
            .set_integration_range(Some(integration_range));
        self
    }

    /// Removes this motion's linear integration range.
    pub const fn without_linear_integration_range(&mut self) -> &mut Self {
        self.linear_controller.set_integration_range(None);
        self
    }

    /// Modifies this motion's linear output limit.
    pub const fn with_linear_output_limit(&mut self, limit: f64) -> &mut Self {
        self.linear_controller.set_output_limit(Some(limit));
        self
    }

    /// Removes this motion's linear output limit.
    pub const fn without_linear_output_limit(&mut self) -> &mut Self {
        self.linear_controller.set_output_limit(None);
        self
    }
}

// MARK: Angular PID Modifiers

impl<M, L, T> MoveToPointFuture<'_, M, L, Pid, T>
where
    M: Arcade,
    L: Feedback<State = f64, Signal = f64> + Unpin,
    T: TracksPosition + TracksHeading + TracksVelocity,
{
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
    pub const fn with_lateral_integration_range(&mut self, integration_range: f64) -> &mut Self {
        self.lateral_controller
            .set_integration_range(Some(integration_range));
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
