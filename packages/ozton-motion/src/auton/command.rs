//! Stateless motion targets and per-command progress.

use std::time::Duration;

use ozton_math::{
    desaturate,
    geometry::{Pose, wheel_speeds, wrap_heading},
    path::{
        Path,
        follower::{self, Follower, Progress},
    },
};
use ozton_tracking::localizer::MAX_RANGES;

#[derive(Debug, Clone, Copy)]
pub struct RangeFeedback {
    pub distance: f64,
    pub timestamp: Duration,
}

/// All feedback available to autonomous on one tick.
#[derive(Debug, Clone, Copy)]
pub struct Feedback {
    pub pose: Pose,
    pub forward_travel: f64,
    pub forward_velocity: f64,
    pub angular_velocity: f64,
    pub ranges: [Option<RangeFeedback>; MAX_RANGES],
    pub reset_accepted: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct DriveOptions {
    pub track_width: f64,
    pub maximum_wheel_speed: f64,
    pub position_tolerance: f64,
    pub heading_tolerance: f64,
    pub speed_tolerance: f64,
    pub angular_speed_tolerance: f64,
    pub settle: Duration,
}

impl DriveOptions {
    /// Settling defaults for a differential robot, in meters and radians.
    pub const fn differential(track_width: f64, maximum_wheel_speed: f64) -> Self {
        Self {
            track_width,
            maximum_wheel_speed,
            position_tolerance: 0.025,
            heading_tolerance: 0.06,
            speed_tolerance: 0.04,
            angular_speed_tolerance: 0.12,
            settle: Duration::from_millis(120),
        }
    }

    pub(super) fn valid(self) -> bool {
        [
            self.track_width,
            self.maximum_wheel_speed,
            self.position_tolerance,
            self.heading_tolerance,
            self.speed_tolerance,
            self.angular_speed_tolerance,
        ]
        .iter()
        .all(|value| value.is_finite() && *value > 0.0)
    }
}

/// Align to a known range and field heading. Waits at zero output during sensor loss.
#[derive(Debug, Clone, Copy)]
pub struct Alignment {
    pub sensor: usize,
    pub distance: f64,
    pub heading: f64,
    pub maximum_speed: f64,
    /// +1 for a front-facing sensor, -1 for a rear-facing sensor.
    pub direction: f64,
}

/// Approach at bounded speed until a debounced switch verifies contact.
pub struct Contact<R> {
    pub known_pose: Pose,
    pub speed: f64,
    pub touched: fn(&R) -> bool,
}

/// A mechanism action at a distance along a path, in meters.
pub type PathMarker<R> = (f64, fn(&mut R));

pub enum Command<'a, R> {
    Follow {
        path: &'a Path,
        follower: Follower,
        reverse: bool,
        /// Trigger once when the monotonic path progress crosses this distance.
        trigger: Option<PathMarker<R>>,
    },
    FollowMarked {
        path: &'a Path,
        follower: Follower,
        reverse: bool,
        /// Triggers run once, in ascending distance order. Equal distances are allowed.
        markers: &'a [PathMarker<R>],
    },
    Turn {
        heading: f64,
    },
    DriveDistance {
        meters: f64,
        heading: f64,
        maximum_speed: f64,
    },
    Align(Alignment),
    Contact(Contact<R>),
    DistanceReset,
    Wait(Duration),
    WaitUntil(fn(&R) -> bool),
}

impl<R> Command<'_, R> {
    pub(super) fn path_start(&self) -> Option<(&Path, bool)> {
        match self {
            Self::Follow { path, reverse, .. } | Self::FollowMarked { path, reverse, .. } => {
                Some((path, *reverse))
            }
            _ => None,
        }
    }

    pub(super) fn valid(&self, options: DriveOptions) -> bool {
        match self {
            Self::Follow {
                path,
                follower,
                trigger,
                ..
            } => {
                follower.is_valid()
                    && (path.limits().track_width - options.track_width).abs() < 1e-6
                    && path.limits().wheel_speed <= options.maximum_wheel_speed
                    && trigger.is_none_or(|(at, _)| (0.0..=path.end().distance).contains(&at))
            }
            Self::FollowMarked {
                path,
                follower,
                markers,
                ..
            } => {
                follower.is_valid()
                    && (path.limits().track_width - options.track_width).abs() < 1e-6
                    && path.limits().wheel_speed <= options.maximum_wheel_speed
                    && markers
                        .iter()
                        .all(|(at, _)| (0.0..=path.end().distance).contains(at))
                    && markers.windows(2).all(|pair| pair[0].0 <= pair[1].0)
            }
            Self::Turn { heading } => heading.is_finite(),
            Self::DriveDistance {
                meters,
                heading,
                maximum_speed,
            } => {
                meters.is_finite()
                    && heading.is_finite()
                    && *maximum_speed > 0.0
                    && *maximum_speed <= options.maximum_wheel_speed
            }
            Self::Align(alignment) => {
                alignment.sensor < MAX_RANGES
                    && alignment.distance.is_finite()
                    && alignment.distance > 0.0
                    && alignment.heading.is_finite()
                    && (0.0..=options.maximum_wheel_speed).contains(&alignment.maximum_speed)
                    && alignment.maximum_speed > 0.0
                    && alignment.direction.abs() == 1.0
            }
            Self::Contact(contact) => {
                contact.known_pose.is_finite()
                    && contact.speed.is_finite()
                    && contact.speed.abs() <= options.maximum_wheel_speed
                    && contact.speed != 0.0
            }
            Self::Wait(_) | Self::WaitUntil(_) | Self::DistanceReset => true,
        }
    }
}

#[derive(Default)]
pub(super) struct State {
    progress: Progress,
    settled: Duration,
    fresh_range: Option<Duration>,
    settled_range: Option<Duration>,
    start_travel: Option<f64>,
    next_marker: usize,
}

pub(super) struct Step {
    pub wheels: [f64; 2],
    pub complete: bool,
    pub progress: f64,
    pub lost: bool,
}

impl State {
    pub(super) fn step<R>(
        &mut self,
        command: &mut Command<'_, R>,
        robot: &mut R,
        feedback: Feedback,
        elapsed: Duration,
        dt: Duration,
        options: DriveOptions,
    ) -> Step {
        let slow = feedback.forward_velocity.abs() <= options.speed_tolerance
            && feedback.angular_velocity.abs() <= options.angular_speed_tolerance;
        let error_at = |heading| wrap_heading(heading - feedback.pose.heading);
        let drive = |linear, angular| {
            desaturate(
                wheel_speeds(linear, angular, options.track_width),
                options.maximum_wheel_speed,
            )
        };
        let mut fresh_alignment_range = false;
        let mut lost = false;
        let (wheels, ready, settle) = match command {
            Command::Follow {
                path,
                follower,
                reverse,
                ..
            }
            | Command::FollowMarked {
                path,
                follower,
                reverse,
                ..
            } => {
                self.progress.update(
                    path,
                    feedback.pose.position,
                    options.maximum_wheel_speed * dt.as_secs_f64() * 2.0,
                );
                let end = path.end();
                let tracking_error = path
                    .at_distance(self.progress.distance())
                    .position
                    .distance(feedback.pose.position);
                lost = tracking_error > 0.35
                    || (matches!(follower, Follower::Ramsete { .. })
                        && elapsed.as_secs_f64() >= end.time
                        && self.progress.distance() < end.distance - 0.20);
                let heading = end.heading + if *reverse { core::f64::consts::PI } else { 0.0 };
                let ready = self.progress.distance() >= end.distance - options.position_tolerance
                    && feedback.pose.position.distance(end.position) <= options.position_tolerance
                    && error_at(heading).abs() <= options.heading_tolerance;
                let wheels = if ready {
                    [0.0; 2]
                } else {
                    follower::command(
                        *follower,
                        path,
                        self.progress,
                        feedback.pose,
                        elapsed.as_secs_f64(),
                        dt.as_secs_f64(),
                        *reverse,
                        options.position_tolerance,
                    )
                };
                if !lost {
                    let distance = if ready {
                        end.distance
                    } else {
                        self.progress.distance()
                    };
                    self.trigger(command, robot, distance);
                }
                (wheels, ready && slow, options.settle)
            }
            Command::Turn { heading } => {
                let error = error_at(*heading);
                let ready = error.abs() <= options.heading_tolerance;
                (
                    drive(0.0, if ready { 0.0 } else { 3.0 * error }),
                    ready && slow,
                    options.settle,
                )
            }
            Command::DriveDistance {
                meters,
                heading,
                maximum_speed,
            } => {
                let start = *self.start_travel.get_or_insert(feedback.forward_travel);
                let error = *meters - (feedback.forward_travel - start);
                let heading = error_at(*heading);
                let reached = error.abs() <= options.position_tolerance;
                let facing = heading.abs() <= options.heading_tolerance;
                let speed = if reached || heading.abs() > 0.35 {
                    0.0
                } else {
                    (2.5 * error).clamp(-*maximum_speed, *maximum_speed)
                };
                (
                    drive(speed, if facing { 0.0 } else { 3.0 * heading }),
                    reached && facing && slow,
                    options.settle,
                )
            }
            Command::Align(alignment) => {
                if let Some(range) = feedback.ranges[alignment.sensor] {
                    fresh_alignment_range = self.fresh_range != Some(range.timestamp);
                    self.fresh_range = Some(range.timestamp);
                    let distance_error = range.distance - alignment.distance;
                    let heading_error = error_at(alignment.heading);
                    let position_ready = distance_error.abs() <= options.position_tolerance;
                    let heading_ready = heading_error.abs() <= options.heading_tolerance;
                    let linear = if position_ready || heading_error.abs() > 0.2 {
                        0.0
                    } else {
                        (2.0 * distance_error * alignment.direction)
                            .clamp(-alignment.maximum_speed, alignment.maximum_speed)
                    };
                    (
                        drive(
                            linear,
                            if heading_ready {
                                0.0
                            } else {
                                3.0 * heading_error
                            },
                        ),
                        position_ready && heading_ready && slow,
                        options.settle,
                    )
                } else {
                    ([0.0; 2], false, options.settle)
                }
            }
            Command::Contact(contact) => {
                let touched = (contact.touched)(robot);
                let heading = error_at(contact.known_pose.heading);
                if touched {
                    (
                        [0.0; 2],
                        slow && heading.abs() <= options.heading_tolerance,
                        options.settle.max(Duration::from_millis(30)),
                    )
                } else {
                    let linear = if heading.abs() < 0.2 {
                        contact.speed
                    } else {
                        0.0
                    };
                    (drive(linear, 3.0 * heading), false, options.settle)
                }
            }
            Command::DistanceReset => ([0.0; 2], feedback.reset_accepted, Duration::ZERO),
            Command::Wait(duration) => ([0.0; 2], elapsed >= *duration, Duration::ZERO),
            Command::WaitUntil(ready) => ([0.0; 2], ready(robot), Duration::ZERO),
        };
        self.settled = if ready {
            self.settled.saturating_add(dt)
        } else {
            Duration::ZERO
        };
        let complete = if matches!(command, Command::Align(_)) {
            if !ready {
                self.settled_range = None;
                false
            } else {
                if fresh_alignment_range && self.settled_range.is_none() {
                    self.settled_range = Some(elapsed);
                }
                fresh_alignment_range
                    && self
                        .settled_range
                        .is_some_and(|start| elapsed - start >= settle)
            }
        } else {
            ready && self.settled >= settle
        };
        Step {
            wheels,
            complete,
            progress: self.progress.distance(),
            lost,
        }
    }

    fn trigger<R>(&mut self, command: &mut Command<'_, R>, robot: &mut R, distance: f64) {
        match command {
            Command::Follow { trigger, .. } => {
                if trigger.is_some_and(|(at, _)| distance >= at)
                    && let Some((_, action)) = trigger.take()
                {
                    action(robot);
                }
            }
            Command::FollowMarked { markers, .. } => {
                while let Some(&(at, action)) = markers.get(self.next_marker) {
                    if distance < at {
                        break;
                    }
                    self.next_marker += 1;
                    action(robot);
                }
            }
            _ => {}
        }
    }
}
