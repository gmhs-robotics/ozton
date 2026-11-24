use std::{
    future::Future,
    pin::Pin,
    task::Poll,
    time::{Duration, Instant},
};

use glam::DVec2 as Vec2;
use ozton_control::{Tolerances, axis::Linear, loops::Feedback};
use ozton_drivetrain::{Drive, HasLocalization};
use ozton_math::{
    geometry::{bearing, wrap_heading},
    localization::observation::{TracksHeading, TracksPosition, TracksVelocity},
    uom::si::{
        angular_velocity::radian_per_second,
        f64::{Angle, AngularVelocity, Velocity},
        velocity::meter_per_second,
    },
};
use ozton_runtime::clock::{Sleep, sleep};

use crate::Arcade;

pub(crate) struct State {
    sleep: Sleep,
    settling: ozton_control::Settling,
    angular_settling: ozton_control::Settling,
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
    pub(crate) final_heading: Option<Angle>,
    pub(crate) angular_tolerances: Tolerances,
    pub(crate) heading_controller: A,
    pub(crate) reverse: bool,
    pub(crate) timeout: Option<Duration>,
    pub(crate) tolerances: Tolerances<Linear>,
    pub(crate) linear_controller: L,
    pub(crate) angular_controller: A,
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
                angular_settling: Default::default(),
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
        let yaw_rate = this
            .final_heading
            .map(|_| this.drivetrain.localization().angular_velocity());
        let feedback_error = if !this.target_point.is_finite()
            || this
                .final_heading
                .is_some_and(|target| !target.value.is_finite())
        {
            Some(crate::MotionError::InvalidTarget)
        } else if !distance_error.is_finite()
            || !heading.value.is_finite()
            || !velocity.is_finite()
            || yaw_rate.is_some_and(|rate| !rate.is_finite())
        {
            Some(crate::MotionError::InvalidFeedback)
        } else {
            None
        };
        if let Some(error) = feedback_error {
            this.state = None;
            return Poll::Ready(stop_with_error(this.drivetrain, error));
        }

        let position_settled = this.tolerances.check_with(
            &mut state.settling,
            ozton_math::qty!((distance_error) m),
            Velocity::new::<meter_per_second>(velocity),
            now,
        );
        let heading_error = this
            .final_heading
            .map(|target| wrap_heading(heading - target));
        let within_position_tolerance = distance_error == 0.0
            || this
                .tolerances
                .error_tolerance
                .is_some_and(|tolerance| distance_error < tolerance.value)
            || (this.final_heading.is_some() && this.tolerances.error_tolerance.is_none());
        let angular_settled = match (heading_error, yaw_rate) {
            (Some(error), Some(rate)) if within_position_tolerance => {
                this.angular_tolerances.check_with(
                    &mut state.angular_settling,
                    error,
                    AngularVelocity::new::<radian_per_second>(rate),
                    now,
                )
            }
            (Some(_), _) => {
                state.angular_settling = Default::default();
                false
            }
            (None, _) => true,
        };
        let settled = position_settled && angular_settled;
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

        let (linear_output, angular_output) = if within_position_tolerance {
            let angular = match heading_error {
                Some(error)
                    if error.value != 0.0
                        && this
                            .angular_tolerances
                            .error_tolerance
                            .is_none_or(|tolerance| error.value.abs() >= tolerance.value) =>
                {
                    this.heading_controller.update(-error.value, 0.0, dt)
                }
                _ => 0.0,
            };
            (0.0, angular)
        } else {
            let drive_heading = if this.reverse {
                heading + ozton_math::qty!((std::f64::consts::PI) rad)
            } else {
                heading
            };
            let angle_error = wrap_heading(drive_heading - bearing(local_target));
            // Keep steering authority close to the goal and when facing away from it.
            let steering_error = angle_error.value * distance_error.max(0.5);
            let angular = this.angular_controller.update(-steering_error, 0.0, dt);
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

        ozton_runtime::log_row!(
            Trace,
            "LEGACY_MOTION_TICK",
            "command=seek,dt_us={},error_m={distance_error:.5},x_m={:.5},y_m={:.5},heading_rad={:.5},forward_mps={velocity:.5},throttle={linear_output:.5},turn={angular_output:.5}",
            dt.as_micros(),
            position.x,
            position.y,
            heading.value
        );
        if let Some(error) = heading_error {
            ozton_runtime::log_row!(
                Trace,
                "SEEK_HEADING",
                "error_rad={:.5},yaw_radps={:.5},position_ready={}",
                error.value,
                yaw_rate.unwrap_or(0.0),
                within_position_tolerance
            );
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
