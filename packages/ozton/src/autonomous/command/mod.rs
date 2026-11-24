//! Stateless motion targets and per-command progress.

use std::time::Duration;

use ozton_localization::{Feedback, MAX_RANGES};
use ozton_math::{
    desaturate,
    geometry::{DVec2, Pose, wheel_speeds, wrap_heading},
    path::Path,
    uom::si::{
        angular_velocity::radian_per_second,
        f64::{Angle, AngularVelocity, Length, Time, Velocity},
        time::second,
        velocity::meter_per_second,
    },
};
use ozton_motion::pursuit::profiled::{self, Follower, FollowerSample, Progress};

use super::DriveOptions;

/// Align to a known range and field heading. Waits at zero output during sensor loss.
#[derive(Debug, Clone, Copy)]
pub struct Alignment {
    /// Zero-based index in the configured range list.
    pub sensor: usize,
    pub distance: Length,
    pub heading: Angle,
    pub maximum_speed: Velocity,
    pub facing: RangeFacing,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RangeFacing {
    Front,
    Rear,
}

impl RangeFacing {
    const fn direction(self) -> f64 {
        match self {
            Self::Front => 1.0,
            Self::Rear => -1.0,
        }
    }
}

/// Approach at bounded speed until a debounced switch verifies contact.
pub struct Contact<R> {
    pub known_pose: Pose,
    pub speed: Velocity,
    pub touched: fn(&R) -> bool,
}

/// A mechanism action at a distance along a path, in meters.
pub struct PathMarker<R> {
    pub distance: Length,
    pub action: fn(&mut R),
}

impl<R> PathMarker<R> {
    pub const fn new(distance: Length, action: fn(&mut R)) -> Self {
        Self { distance, action }
    }
}

/// Evaluates contact evidence using the current pose and elapsed command time.
pub type ContactCondition<'a, R> = &'a mut dyn FnMut(&R, Feedback, Duration) -> bool;

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
        heading: Angle,
    },
    /// Aim toward a surveyed field point while holding translation stopped.
    TurnToPoint {
        target: DVec2,
    },
    /// Closed-loop field-point navigation using full planar pose, including lateral drift.
    DriveToPoint {
        target: DVec2,
        reverse: bool,
        maximum_speed: Velocity,
    },
    DriveDistance {
        meters: Length,
        /// None holds the heading of the first valid feedback sample.
        heading: Option<Angle>,
        maximum_speed: Velocity,
    },
    /// Forward travel at constant curvature, with a positive sweep turning left.
    DriveArc {
        radius: Length,
        sweep: Angle,
        maximum_speed: Velocity,
    },
    Align(Alignment),
    Contact(Contact<R>),
    /// Approach until the caller confirms contact, then brake and settle.
    /// The caller decides whether and how to correct field pose afterward.
    ApproachContact {
        heading: Angle,
        speed: Velocity,
        settle: Duration,
        touched: ContactCondition<'a, R>,
    },
    DistanceReset,
    Wait(Duration),
    WaitUntil(fn(&R) -> bool),
    /// A captured predicate, continuously true while stopped for `stable`.
    WaitFor {
        ready: &'a mut dyn FnMut(&R) -> bool,
        stable: Duration,
    },
}

impl<R> Command<'_, R> {
    pub(super) fn status(&self) -> &'static str {
        match self {
            Self::Turn { .. } | Self::TurnToPoint { .. } => "Autonomous: turn",
            Self::DriveToPoint { .. } => "Autonomous: drive to point",
            Self::DriveDistance { .. } => "Autonomous: drive distance",
            Self::DriveArc { .. } => "Autonomous: drive arc",
            Self::DistanceReset => "Autonomous: distance reset",
            Self::Contact(_) | Self::ApproachContact { .. } => "Autonomous: approach contact",
            Self::Align(_) => "Autonomous: align",
            Self::Follow { .. } | Self::FollowMarked { .. } => "Autonomous: follow path",
            Self::Wait(_) | Self::WaitUntil(_) | Self::WaitFor { .. } => "Autonomous: wait",
        }
    }

    /// Encoder-based travel along a heading in CCW radians.
    pub const fn drive_distance(distance: Length, heading: Angle, maximum_speed: Velocity) -> Self {
        Self::DriveDistance {
            meters: distance,
            heading: Some(heading),
            maximum_speed,
        }
    }

    /// Field target in right/forward order.
    pub const fn drive_to_point(target: DVec2, maximum_speed: Velocity, reverse: bool) -> Self {
        Self::DriveToPoint {
            target,
            maximum_speed,
            reverse,
        }
    }

    pub(super) fn requires_field_pose(&self) -> bool {
        matches!(
            self,
            Self::DriveToPoint { .. }
                | Self::DriveArc { .. }
                | Self::TurnToPoint { .. }
                | Self::Follow { .. }
                | Self::FollowMarked { .. }
                | Self::ApproachContact { .. }
                | Self::DistanceReset
        )
    }
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
                    && (path.limits().track_width.value - options.track_width.value).abs() < 1e-6
                    && path.limits().wheel_speed <= options.maximum_wheel_speed
                    && trigger.as_ref().is_none_or(|marker| {
                        (0.0..=path.end().distance).contains(&marker.distance.value)
                    })
            }
            Self::FollowMarked {
                path,
                follower,
                markers,
                ..
            } => {
                follower.is_valid()
                    && (path.limits().track_width.value - options.track_width.value).abs() < 1e-6
                    && path.limits().wheel_speed <= options.maximum_wheel_speed
                    && markers
                        .iter()
                        .all(|marker| (0.0..=path.end().distance).contains(&marker.distance.value))
                    && markers
                        .windows(2)
                        .all(|pair| pair[0].distance <= pair[1].distance)
            }
            Self::Turn { heading } => heading.value.is_finite(),
            Self::TurnToPoint { target } => target.is_finite(),
            Self::DriveToPoint {
                target,
                maximum_speed,
                ..
            } => {
                target.is_finite()
                    && maximum_speed.value.is_finite()
                    && maximum_speed.value > 0.0
                    && maximum_speed.value <= options.maximum_wheel_speed.value
            }
            Self::DriveDistance {
                meters,
                heading,
                maximum_speed,
            } => {
                meters.value.is_finite()
                    && heading.is_none_or(|angle| angle.value.is_finite())
                    && maximum_speed.value > 0.0
                    && maximum_speed.value <= options.maximum_wheel_speed.value
            }
            Self::DriveArc {
                radius,
                sweep,
                maximum_speed,
            } => {
                radius.value.is_finite()
                    && radius.value >= options.track_width.value / 2.0
                    && sweep.value.is_finite()
                    && sweep.value != 0.0
                    && sweep.value.abs() <= core::f64::consts::TAU
                    && (radius.value * sweep.value).is_finite()
                    && maximum_speed.value > 0.0
                    && maximum_speed.value <= options.maximum_wheel_speed.value
            }
            Self::Align(alignment) => {
                alignment.sensor < MAX_RANGES
                    && alignment.distance.value.is_finite()
                    && alignment.distance.value > 0.0
                    && alignment.heading.value.is_finite()
                    && (0.0..=options.maximum_wheel_speed.value)
                        .contains(&alignment.maximum_speed.value)
                    && alignment.maximum_speed.value > 0.0
            }
            Self::Contact(contact) => {
                contact.known_pose.is_finite()
                    && contact.speed.value.is_finite()
                    && contact.speed.value.abs() <= options.maximum_wheel_speed.value
                    && contact.speed.value != 0.0
            }
            Self::ApproachContact {
                heading,
                speed,
                settle,
                ..
            } => {
                heading.value.is_finite()
                    && speed.value.is_finite()
                    && speed.value.abs() <= options.maximum_wheel_speed.value
                    && speed.value != 0.0
                    && !settle.is_zero()
            }
            Self::Wait(_) | Self::WaitUntil(_) | Self::WaitFor { .. } | Self::DistanceReset => true,
        }
    }
}

pub(super) mod state;
pub(super) use state::State;
