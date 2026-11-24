use std::{
    pin::Pin,
    task::{Context, Poll},
    time::{Duration, Instant},
};

use glam::DVec2 as Vec2;
use ozton_drivetrain::{Drive, HasLocalization};
use ozton_math::{
    localization::observation::{TracksHeading, TracksPosition},
    uom::si::f64::{Angle, Length},
};
use ozton_runtime::clock::{Sleep, sleep};

use super::Waypoint;
use crate::Tank;

pub struct State {
    current: Waypoint,
    next: Waypoint,
    lookahead_point: Vec2,
    start_time: Instant,
    sleep: Sleep,
    terminal: bool,
    previous_position: Vec2,
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
            if !this.lookahead_distance.is_finite() || this.lookahead_distance <= 0.0 {
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

            let Some(mut next) = this.waypoints.next() else {
                return Poll::Ready(
                    this.drivetrain
                        .stop_now()
                        .map_err(crate::MotionError::Stop)
                        .and(Err(crate::MotionError::InvalidPath)),
                );
            };

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
                    next_waypoint
                } else {
                    terminal = true;
                    break;
                };
            }

            // Compute initial lookahead point.
            let lookahead_point = if terminal { next.position } else { match line_segment_circle_intersections(
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
            }};

            this.state.insert(State {
                sleep: sleep(Duration::from_millis(10)),
                start_time: now,
                lookahead_point,
                current,
                next,
                terminal,
                previous_position: position,
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
        let dt = now.saturating_duration_since(state.previous_time).as_secs_f64();
        let speed = position.distance(state.previous_position) / dt.max(1e-6);
        state.previous_position = position;
        state.previous_time = now;

        // If the lookahead circle envelops the end of the current waypoint segment,
        // then switch to the next two waypoints.
        //
        // The ending point of the current segment becomes the starting point of the
        // next and so on until the path is complete.
        while !state.terminal && position.distance(state.next.position) < this.lookahead_distance {
            state.current = state.next;
            state.next = if let Some(next_waypoint) = this.waypoints.next() {
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
                return Poll::Ready(this.drivetrain.stop_now().map_err(crate::MotionError::Stop)
                    .and(Err(crate::MotionError::InvalidFeedback)));
            }
            if distance <= crate::defaults::AUTON_POSITION_TOLERANCE.value && speed <= 0.04 {
                let since = *state.settled_since.get_or_insert(now);
                if now.saturating_duration_since(since) >= Duration::from_millis(120) {
                    this.state = None;
                    return Poll::Ready(crate::finish_motion(this.drivetrain.stop_now(), crate::MotionOutcome::Settled));
                }
            } else {
                state.settled_since = None;
            }
            let error = ozton_math::geometry::wrap_heading(
                ozton_math::geometry::bearing(delta) - heading
            ).value;
            let reverse = *state.terminal_reverse.get_or_insert(error.abs() > core::f64::consts::FRAC_PI_2);
            if error.abs() > 2.3 && speed < 0.04 {
                state.terminal_reverse = Some(!reverse);
            }
            let reverse = state.terminal_reverse.unwrap_or(reverse);
            let error = if reverse {
                ozton_math::geometry::wrap_heading(ozton_math::angle!(error + core::f64::consts::PI, rad)).value
            } else { error };
            let linear = (2.0 * (distance - crate::defaults::AUTON_POSITION_TOLERANCE.value * 0.5).max(0.0)).min(0.35) * error.cos().max(0.0)
                * if reverse { -1.0 } else { 1.0 };
            let (linear, turn) = if distance <= crate::defaults::AUTON_POSITION_TOLERANCE.value { (0.0, 0.0) } else { (linear, (1.5 * error).clamp(-0.4, 0.4)) };
            if let Err(output) = Tank::drive(this.drivetrain, linear - turn, linear + turn) {
                this.state = None;
                return Poll::Ready(Err(crate::motion_output_error(output, this.drivetrain.stop_now())));
            }
            state.sleep = sleep(Duration::from_millis(10));
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
            (None, None) => { state.lookahead_point = state.next.position; }

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

        if let Err(output) = Tank::drive(
            this.drivetrain,
            velocity * (2.0 + curvature * this.track_width) / 2.0,
            velocity * (2.0 - curvature * this.track_width) / 2.0,
        ) {
            this.state = None;
            return Poll::Ready(Err(crate::motion_output_error(
                output,
                this.drivetrain.stop_now(),
            )));
        }

        state.sleep = sleep(Duration::from_millis(10));
        cx.waker().wake_by_ref();
        Poll::Pending
    }
}

// MARK: Modifiers

impl<M, I> PurePursuitFuture<'_, M, I>
where
    M: Drive + HasLocalization<Localization: TracksPosition + TracksHeading>,
    I: Iterator<Item = Waypoint> + Unpin,
{
    /// Modifies this motion's track width.
    pub const fn with_track_width(&mut self, track_width: Length) -> &mut Self {
        self.track_width = track_width.value;
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

    /// Modifies this motion's lookahead distance.
    pub const fn with_lookahead_distance(&mut self, lookahead_distance: Length) -> &mut Self {
        self.lookahead_distance = lookahead_distance.value;
        self
    }
}

// MARK: Math Functions

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
