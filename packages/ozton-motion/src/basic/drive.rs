use std::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
    time::{Duration, Instant},
};

use ozton_control::{
    Tolerances,
    axis::Linear,
    loops::Feedback,
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

        ozton_runtime::log_row!(
            Trace,
            "LEGACY_MOTION_TICK",
            "command=drive,dt_us={},error_m={linear_error:.5},heading_error_rad={:.5},forward_travel_m={forward_travel:.5},heading_rad={:.5},throttle={linear_output:.5},turn={angular_output:.5}",
            dt.as_micros(),
            angular_error.value,
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
