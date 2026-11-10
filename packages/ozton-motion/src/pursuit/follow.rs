use std::{
    pin::Pin,
    task::{Context, Poll},
    time::{Duration, Instant},
};

use glam::DVec2 as Vec2;
use ozton_drivetrain::{Drivetrain, model::Tank};
use ozton_tracking::{TracksHeading, TracksPosition};
use vexide::{
    math::Angle,
    time::{Sleep, sleep},
};

use super::Waypoint;

pub struct State {
    current: Waypoint,
    next: Waypoint,
    lookahead_point: Vec2,
    start_time: Instant,
    sleep: Sleep,
}

/// Moves a drivetrain along a set of discrete waypoints using pure pursuit.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct PurePursuitFuture<'a, M, T, I>
where
    M: Tank,
    T: TracksPosition + TracksHeading,
    I: Iterator<Item = Waypoint> + Unpin,
{
    pub(crate) drivetrain: &'a mut Drivetrain<M, T>,

    /// Internal future state ("local variables").
    pub(crate) state: Option<State>,

    pub(crate) waypoints: I,
    pub(crate) lookahead_distance: f64,
    pub(crate) track_width: f64,
    pub(crate) timeout: Option<Duration>,
}

// MARK: Future Poll

impl<M, T, I> Future for PurePursuitFuture<'_, M, T, I>
where
    M: Tank,
    T: TracksPosition + TracksHeading,
    I: Iterator<Item = Waypoint> + Unpin,
{
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();

        if this.state.is_none() {
            if !this.lookahead_distance.is_finite() || this.lookahead_distance <= 0.0 {
                drop(this.drivetrain.model.drive_tank(0.0, 0.0));
                return Poll::Ready(());
            }
            let now = Instant::now();
            let position = this.drivetrain.tracking.position();

            let Some(mut next) = this.waypoints.next() else {
                drop(this.drivetrain.model.drive_tank(0.0, 0.0));
                return Poll::Ready(()); // path is empty
            };

            // We want an initial intersection to handle the case where the robot
            // starts entirely off the path, so add a "fake waypoint" to the path at
            // the robot's starting position with the first real waypoint's velocity.
            //
            // This works okay for cases where the robot starts positioned reasonably
            // close to the first waypoint in the path.
            let mut current = Waypoint {
                position,
                velocity: next.velocity,
            };

            // Keep iterating line segments until we find one we haven't intersected.
            while position.distance(next.position) < this.lookahead_distance {
                current = next;
                next = if let Some(next_waypoint) = this.waypoints.next() {
                    next_waypoint
                } else {
                    drop(this.drivetrain.model.drive_tank(0.0, 0.0));
                    return Poll::Ready(());
                };
            }

            // Compute initial lookahead point.
            let lookahead_point = match line_segment_circle_intersections(
                position,
                this.lookahead_distance,
                current.position,
                next.position,
            ) {
                (None, None) => {
                    drop(this.drivetrain.model.drive_tank(0.0, 0.0));
                    return Poll::Ready(());
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
            };

            this.state = Some(State {
                sleep: sleep(Duration::from_millis(5)),
                start_time: now,
                lookahead_point,
                current,
                next,
            });
        }

        let state = this.state.as_mut().unwrap();

        if Pin::new(&mut state.sleep).poll(cx).is_pending() {
            return Poll::Pending;
        }

        // Cancel movement if timeout has elapsed.
        if this
            .timeout
            .is_some_and(|timeout| state.start_time.elapsed() > timeout)
        {
            drop(this.drivetrain.model.drive_tank(0.0, 0.0));
            return Poll::Ready(());
        }

        // Tracking data (robot position and heading)
        let position = this.drivetrain.tracking.position();
        let heading = this.drivetrain.tracking.heading();

        // If the lookahead circle envelops the end of the current waypoint segment,
        // then switch to the next two waypoints.
        //
        // The ending point of the current segment becomes the starting point of the
        // next and so on until the path is complete.
        while position.distance(state.next.position) < this.lookahead_distance {
            state.current = state.next;
            state.next = if let Some(next_waypoint) = this.waypoints.next() {
                next_waypoint
            } else {
                // We're out of waypoints, meaning the end of path has been reached.
                drop(this.drivetrain.model.drive_tank(0.0, 0.0));
                return Poll::Ready(());
            };
        }

        // Compute lookahead point.
        match line_segment_circle_intersections(
            position,
            this.lookahead_distance,
            state.current.position,
            state.next.position,
        ) {
            // No intersections; the lookahead circle isn't intersecting the path.
            (None, None) => {}

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

        // Take the profiled velocity of the closest point to the robot on the path.
        let velocity =
            if state.current.position.distance(position) < state.next.position.distance(position) {
                state.current.velocity
            } else {
                state.next.velocity
            };

        let curvature = signed_arc_curvature(position, heading, state.lookahead_point);

        drop(this.drivetrain.model.drive_tank(
            velocity * (2.0 + curvature * this.track_width) / 2.0,
            velocity * (2.0 - curvature * this.track_width) / 2.0,
        ));

        state.sleep = sleep(Duration::from_millis(5));
        cx.waker().wake_by_ref();
        Poll::Pending
    }
}

// MARK: Modifiers

impl<M, T, I> PurePursuitFuture<'_, M, T, I>
where
    M: Tank,
    T: TracksPosition + TracksHeading,
    I: Iterator<Item = Waypoint> + Unpin,
{
    /// Modifies this motion's track width.
    pub const fn with_track_width(&mut self, track_width: f64) -> &mut Self {
        self.track_width = track_width;
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
    pub const fn with_lookahead_distance(&mut self, lookahead_distance: f64) -> &mut Self {
        self.lookahead_distance = lookahead_distance;
        self
    }
}

// MARK: Math Functions

fn signed_arc_curvature(start: Vec2, start_angle: Angle, end: Vec2) -> f64 {
    use ozton_math::geometry::{Pose, pursuit_curvature};

    let pose = Pose {
        position: start,
        heading: start_angle.as_radians(),
    };
    -pursuit_curvature(pose.to_local(end))
}

#[cfg(test)]
mod tests {
    use glam::DVec2;
    use vexide::math::Angle;

    use super::signed_arc_curvature;

    #[test]
    fn curvature_at_quarter_turn_is_finite() {
        let curvature =
            signed_arc_curvature(DVec2::ZERO, Angle::from_degrees(90.0), DVec2::new(1.0, 0.0));
        assert!(curvature.is_finite());
        assert!((curvature - 2.0).abs() < 1e-10);
    }
}

/// Finds the intersection points between a line segment and a circle.
fn line_segment_circle_intersections(
    center: Vec2,
    radius: f64,
    start: Vec2,
    end: Vec2,
) -> (Option<Vec2>, Option<Vec2>) {
    let [first, last] = ozton_math::geometry::circle_segment_fractions(center, radius, start, end)
        .map(|fraction| fraction.map(|value| start.lerp(end, value)));
    (first, last)
}
