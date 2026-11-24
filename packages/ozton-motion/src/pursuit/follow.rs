use std::{
    pin::Pin,
    task::{Context, Poll},
    time::{Duration, Instant},
};

use glam::DVec2 as Vec2;
use ozton_drivetrain::{Drive, HasLocalization};
use ozton_math::{
    localization::observation::{TracksHeading, TracksPosition},
    path::Waypoint,
    uom::si::f64::Angle,
};
use ozton_runtime::clock::{Sleep, sleep};

use super::PursuitTerminalConfig;
use crate::Tank;

pub struct State {
    current: Waypoint,
    next: Waypoint,
    lookahead_point: Vec2,
    start_time: Instant,
    sleep: Sleep,
    terminal: bool,
    previous_position: Vec2,
    previous_heading: Angle,
    previous_time: Instant,
    settled_since: Option<Instant>,
    terminal_reverse: Option<bool>,
}

/// Moves a drivetrain along a set of discrete waypoints using pure pursuit.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct PurePursuitFuture<'a, M, I>
where
    M: Drive + HasLocalization<Localization: TracksPosition + TracksHeading>,
    I: Iterator<Item = Waypoint> + Unpin,
{
    pub(crate) drivetrain: &'a mut M,

    /// Internal future state ("local variables").
    pub(crate) state: Option<State>,

    pub(crate) waypoints: I,
    pub(crate) lookahead_distance: f64,
    pub(crate) track_width: f64,
    pub(crate) timeout: Option<Duration>,
    pub(crate) terminal_config: PursuitTerminalConfig,
}

// MARK: Future Poll

impl<M, I> Drop for PurePursuitFuture<'_, M, I>
where
    M: Drive + HasLocalization<Localization: TracksPosition + TracksHeading>,
    I: Iterator<Item = Waypoint> + Unpin,
{
    fn drop(&mut self) {
        if self.state.is_some() {
            let _ = self.drivetrain.stop_now();
        }
    }
}

impl<M, I> Future for PurePursuitFuture<'_, M, I>
where
    M: Drive + HasLocalization<Localization: TracksPosition + TracksHeading>,
    I: Iterator<Item = Waypoint> + Unpin,
{
    type Output = Result<crate::MotionOutcome, crate::MotionError<M::Error>>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();

        let state = if let Some(state) = &mut this.state {
            state
        } else {
            if !this.lookahead_distance.is_finite()
                || this.lookahead_distance <= 0.0
                || !this.track_width.is_finite()
                || this.track_width <= 0.0
                || !this.terminal_config.valid()
            {
                return Poll::Ready(
                    this.drivetrain
                        .stop_now()
                        .map_err(crate::MotionError::Stop)
                        .and(Err(crate::MotionError::InvalidPath)),
                );
            }
            let now = Instant::now();
            let mut terminal = false;
            let position = this.drivetrain.localization().position();
            let heading = this.drivetrain.localization().heading();

            let Some(mut next) = this.waypoints.next() else {
                return Poll::Ready(
                    this.drivetrain
                        .stop_now()
                        .map_err(crate::MotionError::Stop)
                        .and(Err(crate::MotionError::InvalidPath)),
                );
            };

            if !valid_waypoint(next) || !position.is_finite() || !heading.value.is_finite() {
                return Poll::Ready(
                    this.drivetrain
                        .stop_now()
                        .map_err(crate::MotionError::Stop)
                        .and(Err(crate::MotionError::InvalidPath)),
                );
            }

            // We want an initial intersection to handle the case where the robot
            // starts entirely off the path, so add a "fake waypoint" to the path at
            // the robot's starting position with the first real waypoint's velocity.
            //
            // This works okay for cases where the robot starts positioned reasonably
            // close to the first waypoint in the path.
            let mut current = Waypoint {
                position,
                speed_fraction: next.speed_fraction,
            };

            // Keep iterating line segments until we find one we haven't intersected.
            while position.distance(next.position) < this.lookahead_distance {
                current = next;
                next = if let Some(next_waypoint) = this.waypoints.next() {
                    if !valid_waypoint(next_waypoint) {
                        return Poll::Ready(
                            this.drivetrain
                                .stop_now()
                                .map_err(crate::MotionError::Stop)
                                .and(Err(crate::MotionError::InvalidPath)),
                        );
                    }
                    next_waypoint
                } else {
                    terminal = true;
                    break;
                };
            }

            // Compute initial lookahead point.
            let lookahead_point = if terminal {
                next.position
            } else {
                match line_segment_circle_intersections(
                    position,
                    this.lookahead_distance,
                    current.position,
                    next.position,
                ) {
                    (None, None) => {
                        return Poll::Ready(
                            this.drivetrain
                                .stop_now()
                                .map_err(crate::MotionError::Stop)
                                .and(Err(crate::MotionError::InvalidPath)),
                        );
                    }

                    // One intersection; use that.
                    (Some(solution), None) | (None, Some(solution)) => solution,

                    // Two intersections, pick whichever one is closest to the next point on the path.
                    (Some(solution_1), Some(solution_2)) => {
                        if solution_1.distance(next.position) < solution_2.distance(next.position) {
                            solution_1
                        } else {
                            solution_2
                        }
                    }
                }
            };

            this.state.insert(State {
                sleep: sleep(this.terminal_config.period),
                start_time: now,
                lookahead_point,
                current,
                next,
                terminal,
                previous_position: position,
                previous_heading: heading,
                previous_time: now,
                settled_since: None,
                terminal_reverse: None,
            })
        };

        if Pin::new(&mut state.sleep).poll(cx).is_pending() {
            return Poll::Pending;
        }

        // Cancel movement if timeout has elapsed.
        if this
            .timeout
            .is_some_and(|timeout| state.start_time.elapsed() > timeout)
        {
            this.state = None;
            return Poll::Ready(crate::finish_motion(
                this.drivetrain.stop_now(),
                crate::MotionOutcome::TimedOut,
            ));
        }

        // Tracking data (robot position and heading)
        let position = this.drivetrain.localization().position();
        let heading = this.drivetrain.localization().heading();
        let now = Instant::now();
        let dt = now
            .saturating_duration_since(state.previous_time)
            .as_secs_f64();
        let displacement = position - state.previous_position;
        let speed = displacement.length() / dt.max(1e-6);
        let forward_rate =
            displacement.dot(ozton_math::geometry::direction(heading)) / dt.max(1e-6);
        let yaw_rate = ozton_math::geometry::wrap_heading(heading - state.previous_heading).value
            / dt.max(1e-6);
        state.previous_heading = heading;
        state.previous_position = position;
        if !position.is_finite()
            || !heading.value.is_finite()
            || !speed.is_finite()
            || !yaw_rate.is_finite()
        {
            this.state = None;
            return Poll::Ready(
                this.drivetrain
                    .stop_now()
                    .map_err(crate::MotionError::Stop)
                    .and(Err(crate::MotionError::InvalidFeedback)),
            );
        }
        state.previous_time = now;

        // If the lookahead circle envelops the end of the current waypoint segment,
        // then switch to the next two waypoints.
        //
        // The ending point of the current segment becomes the starting point of the
        // next and so on until the path is complete.
        while !state.terminal && position.distance(state.next.position) < this.lookahead_distance {
            state.current = state.next;
            state.next = if let Some(next_waypoint) = this.waypoints.next() {
                if !valid_waypoint(next_waypoint) {
                    this.state = None;
                    return Poll::Ready(
                        this.drivetrain
                            .stop_now()
                            .map_err(crate::MotionError::Stop)
                            .and(Err(crate::MotionError::InvalidPath)),
                    );
                }
                next_waypoint
            } else {
                state.terminal = true;
                break;
            };
        }

        if state.terminal {
            let delta = state.next.position - position;
            let distance = delta.length();
            if !distance.is_finite() || !heading.value.is_finite() {
                this.state = None;
                return Poll::Ready(
                    this.drivetrain
                        .stop_now()
                        .map_err(crate::MotionError::Stop)
                        .and(Err(crate::MotionError::InvalidFeedback)),
                );
            }
            let config = &mut this.terminal_config;
            let reached = distance <= config.position_tolerance.value;
            let slow = speed <= config.velocity_tolerance.value
                && yaw_rate.abs() <= config.angular_velocity_tolerance.value;
            if reached && slow {
                let since = *state.settled_since.get_or_insert(now);
                if now.saturating_duration_since(since) >= config.settle {
                    this.state = None;
                    return Poll::Ready(crate::finish_motion(
                        this.drivetrain.stop_now(),
                        crate::MotionOutcome::Settled,
                    ));
                }
            } else {
                state.settled_since = None;
            }
            let bearing =
                ozton_math::geometry::wrap_heading(ozton_math::geometry::bearing(delta) - heading)
                    .value;
            let reverse = state
                .terminal_reverse
                .get_or_insert(bearing.abs() > core::f64::consts::FRAC_PI_2);
            let travel_error = |reverse: bool| {
                ozton_math::geometry::wrap_heading(ozton_math::angle!(
                    bearing + if reverse { core::f64::consts::PI } else { 0.0 },
                    rad
                ))
                .value
            };
            let mut error = travel_error(*reverse);
            if error.abs() > 2.3 && slow && !reached {
                *reverse = !*reverse;
                error = travel_error(*reverse);
                config.linear.reset();
                config.angular.reset();
            }
            let dt = Duration::from_secs_f64(dt);
            let linear_error = if reached {
                0.0
            } else {
                distance * error.cos().max(0.0) * if *reverse { -1.0 } else { 1.0 }
            };
            let linear = config.linear.update_with_rate(
                linear_error,
                -forward_rate,
                dt,
                config.rate_filter,
                0.0,
            );
            let turn = config.angular.update_with_rate(
                if reached { 0.0 } else { error },
                -yaw_rate,
                dt,
                config.rate_filter,
                0.0,
            );
            let [left, right] = ozton_math::desaturate([linear - turn, linear + turn], 1.0);
            ozton_runtime::log_row!(
                Trace,
                "LEGACY_PURSUIT_TICK",
                "terminal=true,x_m={:.5},y_m={:.5},heading_rad={:.5},error_m={distance:.5},heading_error_rad={error:.5},forward_mps={forward_rate:.5},yaw_radps={yaw_rate:.5},left={left:.5},right={right:.5}",
                position.x,
                position.y,
                heading.value
            );
            if let Err(output) = Tank::drive(this.drivetrain, left, right) {
                this.state = None;
                return Poll::Ready(Err(crate::motion_output_error(
                    output,
                    this.drivetrain.stop_now(),
                )));
            }
            state.sleep = sleep(this.terminal_config.period);
            cx.waker().wake_by_ref();
            return Poll::Pending;
        }

        // Compute lookahead point.
        match line_segment_circle_intersections(
            position,
            this.lookahead_distance,
            state.current.position,
            state.next.position,
        ) {
            // No intersections; the lookahead circle isn't intersecting the path.
            (None, None) => {
                state.lookahead_point = state.next.position;
            }

            // One intersection; use that.
            (Some(solution), None) | (None, Some(solution)) => state.lookahead_point = solution,

            // Two intersections; pick whichever one is closest to the next point on the path.
            (Some(solution_1), Some(solution_2)) => {
                state.lookahead_point = if solution_1.distance(state.next.position)
                    < solution_2.distance(state.next.position)
                {
                    solution_1
                } else {
                    solution_2
                };
            }
        };

        // Take the profiled speed fraction of the closest point to the robot on the path.
        let velocity =
            if state.current.position.distance(position) < state.next.position.distance(position) {
                state.current.speed_fraction
            } else {
                state.next.speed_fraction
            };

        let curvature = signed_arc_curvature(position, heading, state.lookahead_point);

        let [left, right] = ozton_math::desaturate(
            [
                velocity * (2.0 + curvature * this.track_width) / 2.0,
                velocity * (2.0 - curvature * this.track_width) / 2.0,
            ],
            1.0,
        );
        ozton_runtime::log_row!(
            Trace,
            "LEGACY_PURSUIT_TICK",
            "terminal=false,x_m={:.5},y_m={:.5},heading_rad={:.5},lookahead_x_m={:.5},lookahead_y_m={:.5},forward_mps={forward_rate:.5},yaw_radps={yaw_rate:.5},left={left:.5},right={right:.5}",
            position.x,
            position.y,
            heading.value,
            state.lookahead_point.x,
            state.lookahead_point.y
        );
        if let Err(output) = Tank::drive(this.drivetrain, left, right) {
            this.state = None;
            return Poll::Ready(Err(crate::motion_output_error(
                output,
                this.drivetrain.stop_now(),
            )));
        }

        state.sleep = sleep(this.terminal_config.period);
        cx.waker().wake_by_ref();
        Poll::Pending
    }
}

// MARK: Math Functions

fn valid_waypoint(waypoint: Waypoint) -> bool {
    waypoint.position.is_finite() && (0.0..=1.0).contains(&waypoint.speed_fraction)
}

fn signed_arc_curvature(start: Vec2, start_angle: Angle, end: Vec2) -> f64 {
    use ozton_math::geometry::{Pose, pursuit_curvature};

    let pose = Pose {
        position: start,
        heading: start_angle,
    };
    -pursuit_curvature(pose.to_local(end))
}

/// Finds the intersection points between a line segment and a circle.
fn line_segment_circle_intersections(
    center: Vec2,
    radius: f64,
    start: Vec2,
    end: Vec2,
) -> (Option<Vec2>, Option<Vec2>) {
    let [first, last] = ozton_math::geometry::circle_segment_fractions(
        center,
        ozton_math::length!(radius, m),
        start,
        end,
    )
    .map(|fraction| fraction.map(|value| start.lerp(end, value)));
    (first, last)
}
