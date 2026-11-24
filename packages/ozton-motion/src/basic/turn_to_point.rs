use std::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
    time::{Duration, Instant},
};

use glam::DVec2 as Vec2;
use ozton_control::{Tolerances, axis::Linear, loops::Feedback};
use ozton_drivetrain::{Drive, HasLocalization};
use ozton_math::{
    geometry::wrap_heading,
    localization::observation::{
        TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity,
    },
    uom::{
        ConstZero,
        si::{
            angular_velocity::radian_per_second,
            f64::{Angle, AngularVelocity, Velocity},
            velocity::meter_per_second,
        },
    },
};
use ozton_runtime::clock::{Sleep, sleep};

use crate::Arcade;

pub(crate) struct State {
    sleep: Sleep,
    initial_forward_travel: f64,
    start_time: Instant,
    prev_time: Instant,
    linear_settling: ozton_control::Settling,
    angular_settling: ozton_control::Settling,
}

/// Turns the robot to face a point on the field.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct TurnToPointFuture<'a, M, L, A>
where
    M: Drive,
    L: Feedback<State = f64, Signal = f64> + Unpin,
    A: Feedback<State = Angle, Signal = f64> + Unpin,
    M: HasLocalization<
        Localization: TracksForwardTravel + TracksPosition + TracksHeading + TracksVelocity,
    >,
{
    pub(crate) point: Vec2,
    pub(crate) timeout: Option<Duration>,
    pub(crate) linear_tolerances: Tolerances<Linear>,
    pub(crate) angular_tolerances: Tolerances,
    pub(crate) linear_controller: L,
    pub(crate) angular_controller: A,
    pub(crate) drivetrain: &'a mut M,

    /// Internal future state ("local variables").
    pub(crate) state: Option<State>,
}

// MARK: Future Poll

impl<M, L, A> Drop for TurnToPointFuture<'_, M, L, A>
where
    M: Drive,
    L: Feedback<State = f64, Signal = f64> + Unpin,
    A: Feedback<State = Angle, Signal = f64> + Unpin,
    M: HasLocalization<
        Localization: TracksForwardTravel + TracksHeading + TracksVelocity + TracksPosition,
    >,
{
    fn drop(&mut self) {
        if self.state.is_some() {
            let _ = self.drivetrain.stop_now();
        }
    }
}

impl<M, L, A> Future for TurnToPointFuture<'_, M, L, A>
where
    M: Drive,
    L: Feedback<State = f64, Signal = f64> + Unpin,
    A: Feedback<State = Angle, Signal = f64> + Unpin,
    M: HasLocalization<
        Localization: TracksForwardTravel + TracksHeading + TracksVelocity + TracksPosition,
    >,
{
    type Output = Result<crate::MotionOutcome, crate::MotionError<M::Error>>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();
        let state = this.state.get_or_insert_with(|| {
            let now = Instant::now();
            State {
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
        let position = this.drivetrain.localization().position();
        let heading = this.drivetrain.localization().heading();
        let to_target = this.point - position;
        let target_heading = ozton_math::geometry::bearing(to_target);

        let linear_error = state.initial_forward_travel - forward_travel;
        let angular_error = wrap_heading(heading - target_heading);

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

        let linear_output =
            this.linear_controller
                .update(forward_travel, state.initial_forward_travel, dt);
        let angular_output = this
            .angular_controller
            .update(-angular_error, Angle::ZERO, dt);

        ozton_runtime::log_row!(
            Trace,
            "LEGACY_MOTION_TICK",
            "command=turn_to_point,dt_us={},error_m={linear_error:.5},heading_error_rad={:.5},x_m={:.5},y_m={:.5},heading_rad={:.5},throttle={linear_output:.5},turn={angular_output:.5}",
            dt.as_micros(),
            angular_error.value,
            position.x,
            position.y,
            heading.value
        );
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
