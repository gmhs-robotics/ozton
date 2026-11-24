use std::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
    time::{Duration, Instant},
};

use ozton_control::{
    Tolerances,
    axis::Linear,
    loops::{AngularPid, Feedback, Pid},
};
use ozton_drivetrain::{Drive, HasLocalization};
use ozton_math::{
    geometry::wrap_heading,
    localization::observation::{TracksForwardTravel, TracksHeading, TracksVelocity},
    uom::si::{
        angular_velocity::radian_per_second,
        f64::{Angle, AngularVelocity, Length, Velocity},
        velocity::meter_per_second,
    },
};
use ozton_runtime::clock::{Sleep, sleep};

use crate::Arcade;

pub(crate) struct DriveState {
    pub sleep: Sleep,
    pub initial_forward_travel: f64,
    pub start_time: Instant,
    pub prev_time: Instant,
    pub linear_settling: ozton_control::Settling,
    pub angular_settling: ozton_control::Settling,
}

/// Drives the robot forward or backwards for a distance at a given heading.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct DriveFuture<'a, M, L, A>
where
    M: Drive,
    L: Feedback<State = f64, Signal = f64> + Unpin,
    A: Feedback<State = Angle, Signal = f64> + Unpin,
    M: HasLocalization<Localization: TracksForwardTravel + TracksHeading + TracksVelocity>,
{
    pub(crate) target_distance: f64,
    pub(crate) target_heading: Angle,
    pub(crate) timeout: Option<Duration>,
    pub(crate) linear_tolerances: Tolerances<Linear>,
    pub(crate) angular_tolerances: Tolerances,
    pub(crate) linear_controller: L,
    pub(crate) angular_controller: A,
    pub(crate) drivetrain: &'a mut M,

    /// Internal future state ("local variables").
    pub(crate) state: Option<DriveState>,
}

// MARK: Future Poll

impl<M, L, A> Drop for DriveFuture<'_, M, L, A>
where
    M: Drive,
    L: Feedback<State = f64, Signal = f64> + Unpin,
    A: Feedback<State = Angle, Signal = f64> + Unpin,
    M: HasLocalization<Localization: TracksForwardTravel + TracksHeading + TracksVelocity>,
{
    fn drop(&mut self) {
        if self.state.is_some() {
            let _ = self.drivetrain.stop_now();
        }
    }
}

impl<M, L, A> Future for DriveFuture<'_, M, L, A>
where
    M: Drive,
    L: Feedback<State = f64, Signal = f64> + Unpin,
    A: Feedback<State = Angle, Signal = f64> + Unpin,
    M: HasLocalization<Localization: TracksForwardTravel + TracksHeading + TracksVelocity>,
{
    type Output = Result<crate::MotionOutcome, crate::MotionError<M::Error>>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();
        let state = this.state.get_or_insert_with(|| {
            let now = Instant::now();
            DriveState {
                sleep: sleep(Duration::from_millis(10)),
                initial_forward_travel: this.drivetrain.localization().forward_travel(),
                start_time: now,
                prev_time: now,
                linear_settling: Default::default(),
                angular_settling: Default::default(),
            }
        });

        if Pin::new(&mut state.sleep).poll(cx).is_pending() {
            return Poll::Pending;
        }

        let now = Instant::now();
        let dt = now.saturating_duration_since(state.prev_time);

        let forward_travel = this.drivetrain.localization().forward_travel();
        let heading = this.drivetrain.localization().heading();

        let linear_error = (this.target_distance + state.initial_forward_travel) - forward_travel;
        let angular_error = wrap_heading(this.target_heading - heading);

        // println!("{}", linear_error);

        let linear_settled = this.linear_tolerances.check_with(
            &mut state.linear_settling,
            ozton_math::length!(linear_error, m),
            Velocity::new::<meter_per_second>(this.drivetrain.localization().linear_velocity()),
            now,
        );
        let angular_settled = this.angular_tolerances.check_with(
            &mut state.angular_settling,
            angular_error,
            AngularVelocity::new::<radian_per_second>(
                this.drivetrain.localization().angular_velocity(),
            ),
            now,
        );

        let settled = linear_settled && angular_settled;
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

        let linear_output = this.linear_controller.update(
            forward_travel,
            this.target_distance + state.initial_forward_travel,
            dt,
        );
        let angular_output = -this
            .angular_controller
            .update(heading, this.target_heading, dt);

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

// MARK: Generic Modifiers

impl<M, L, A> DriveFuture<'_, M, L, A>
where
    M: Drive,
    L: Feedback<State = f64, Signal = f64> + Unpin,
    A: Feedback<State = Angle, Signal = f64> + Unpin,
    M: HasLocalization<Localization: TracksForwardTravel + TracksHeading + TracksVelocity>,
{
    /// Modifies this motion's linear feedback controller.
    pub fn with_linear_controller(&mut self, controller: L) -> &mut Self {
        self.linear_controller = controller;
        self
    }

    /// Modifies this motion's angular feedback controller.
    pub fn with_angular_controller(&mut self, controller: A) -> &mut Self {
        self.angular_controller = controller;
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

    /// Modifies this motion's linear tolerances.
    pub const fn with_linear_tolerances(&mut self, tolerances: Tolerances<Linear>) -> &mut Self {
        self.linear_tolerances = tolerances;
        self
    }

    /// Modifies this motion's linear error tolerance.
    pub const fn with_linear_error_tolerance(&mut self, tolerance: Length) -> &mut Self {
        self.linear_tolerances.error_tolerance = Some(tolerance);
        self
    }

    /// Removes this motion's linear error tolerance.
    pub const fn without_linear_error_tolerance(&mut self) -> &mut Self {
        self.linear_tolerances.error_tolerance = None;
        self
    }

    /// Modifies this motion's linear velocity tolerance.
    pub const fn with_linear_velocity_tolerance(&mut self, tolerance: Velocity) -> &mut Self {
        self.linear_tolerances.velocity_tolerance = Some(tolerance);
        self
    }

    /// Removes this motion's linear velocity tolerance.
    pub const fn without_linear_velocity_tolerance(&mut self) -> &mut Self {
        self.linear_tolerances.velocity_tolerance = None;
        self
    }

    /// Modifies this motion's linear tolerance duration.
    pub const fn with_linear_tolerance_duration(&mut self, duration: Duration) -> &mut Self {
        self.linear_tolerances.duration = Some(duration);
        self
    }

    /// Removes this motion's linear tolerance duration.
    pub const fn without_linear_tolerance_duration(&mut self) -> &mut Self {
        self.linear_tolerances.duration = None;
        self
    }

    /// Removes this motion's linear and angular tolerance durations.
    pub const fn without_tolerance_duration(&mut self) -> &mut Self {
        self.linear_tolerances.duration = None;
        self.angular_tolerances.duration = None;
        self
    }

    /// Modifies this motion's angular tolerances.
    pub const fn with_angular_tolerances(&mut self, tolerances: Tolerances) -> &mut Self {
        self.angular_tolerances = tolerances;
        self
    }

    /// Modifies this motion's angular error tolerance.
    pub const fn with_angular_error_tolerance(&mut self, tolerance: Angle) -> &mut Self {
        self.angular_tolerances.error_tolerance = Some(tolerance);
        self
    }

    /// Removes this motion's angular error tolerance.
    pub const fn without_angular_error_tolerance(&mut self) -> &mut Self {
        self.angular_tolerances.error_tolerance = None;
        self
    }

    /// Modifies this motion's angular velocity tolerance.
    pub const fn with_angular_velocity_tolerance(
        &mut self,
        tolerance: AngularVelocity,
    ) -> &mut Self {
        self.angular_tolerances.velocity_tolerance = Some(tolerance);
        self
    }

    /// Removes this motion's angular velocity tolerance.
    pub const fn without_angular_velocity_tolerance(&mut self) -> &mut Self {
        self.angular_tolerances.velocity_tolerance = None;
        self
    }

    /// Modifies this motion's angular tolerance duration.
    pub const fn with_angular_tolerance_duration(&mut self, duration: Duration) -> &mut Self {
        self.angular_tolerances.duration = Some(duration);
        self
    }

    /// Removes this motion's angular tolerance duration.
    pub const fn without_angular_tolerance_duration(&mut self) -> &mut Self {
        self.angular_tolerances.duration = None;
        self
    }
}

// MARK: Linear PID Modifiers

impl<M, A> DriveFuture<'_, M, Pid, A>
where
    M: Drive,
    A: Feedback<State = Angle, Signal = f64> + Unpin,
    M: HasLocalization<Localization: TracksForwardTravel + TracksHeading + TracksVelocity>,
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

impl<M, L> DriveFuture<'_, M, L, AngularPid>
where
    M: Drive,
    L: Feedback<State = f64, Signal = f64> + Unpin,
    M: HasLocalization<Localization: TracksForwardTravel + TracksHeading + TracksVelocity>,
{
    /// Modifies this motion's angular PID gains.
    pub const fn with_angular_gains(&mut self, kp: f64, ki: f64, kd: f64) -> &mut Self {
        self.angular_controller.set_gains(kp, ki, kd);
        self
    }

    /// Modifies this motion's angular proportional gain (`kp`).
    pub const fn with_angular_kp(&mut self, kp: f64) -> &mut Self {
        self.angular_controller.set_kp(kp);
        self
    }

    /// Modifies this motion's angular integral gain (`ki`).
    pub const fn with_angular_ki(&mut self, ki: f64) -> &mut Self {
        self.angular_controller.set_ki(ki);
        self
    }

    /// Modifies this motion's angular derivative gain (`kd`).
    pub const fn with_angular_kd(&mut self, kd: f64) -> &mut Self {
        self.angular_controller.set_kd(kd);
        self
    }

    /// Modifies this motion's angular integration range.
    pub const fn with_angular_integration_range(&mut self, integration_range: Angle) -> &mut Self {
        self.angular_controller
            .set_integration_range(Some(integration_range));
        self
    }

    /// Modifies this motion's angular output limit.
    pub const fn with_angular_output_limit(&mut self, limit: f64) -> &mut Self {
        self.angular_controller.set_output_limit(Some(limit));
        self
    }

    /// Removes this motion's angular integration range.
    pub const fn without_angular_integration_range(&mut self) -> &mut Self {
        self.angular_controller.set_integration_range(None);
        self
    }

    /// Removes this motion's angular output limit.
    pub const fn without_angular_output_limit(&mut self) -> &mut Self {
        self.angular_controller.set_output_limit(None);
        self
    }
}
