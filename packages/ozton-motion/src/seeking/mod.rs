//! Point-to-point feedback seeking.

use std::time::Duration;

use glam::DVec2 as Vec2;
use ozton_control::{Tolerances, axis::Linear, loops::Feedback};
use ozton_drivetrain::{Drivetrain, model::Arcade};
use ozton_math::uom::si::f64::Length;
use ozton_tracking::{Tracking, TracksHeading, TracksPosition, TracksVelocity};

mod move_to_point;

pub use move_to_point::MoveToPointFuture;

/// Point-to-point feedback seeking.
///
/// This struct provides implementations of adaptive feedback seeking algorithms, which
/// utilize two feedback controllers (one for straight driving and one for turning) to
/// reach a desired point. This is most commonly done using two PID controllers.
///
/// Seeking motions include:
/// - [`move_to_point`](Seeking::move_to_point), which moves the drivetrain to a desired point.
#[derive(PartialEq)]
pub struct Seeking<L, A>
where
    L: Feedback<State = f64, Signal = f64> + Unpin + Clone,
    A: Feedback<State = f64, Signal = f64> + Unpin + Clone,
{
    /// Linear (forward) feedback controller.
    pub linear_controller: L,

    /// Angular (cross-track) feedback controller.
    pub lateral_controller: A,

    /// Linear settling conditions.
    ///
    /// Error is denoted by the distance from the target, while velocity
    /// is the robot's linear forward velocity.
    pub tolerances: Tolerances<Linear>,

    /// Maximum duration the motion can take before being cancelled.
    pub timeout: Option<Duration>,
}

impl<L, A> Seeking<L, A>
where
    L: Feedback<State = f64, Signal = f64> + Unpin + Clone,
    A: Feedback<State = f64, Signal = f64> + Unpin + Clone,
{
    /// Moves the robot to a 2D point.
    ///
    /// The final heading of the robot after this motion executes is undefined.
    pub fn move_to_point<
        'a,
        M: Arcade,
        S: Tracking,
        T: TracksPosition + TracksHeading + TracksVelocity,
        D,
    >(
        &self,
        drivetrain: &'a mut Drivetrain<M, S, T, D>,
        point: [Length; 2],
    ) -> MoveToPointFuture<'a, M, L, A, S, T, D> {
        MoveToPointFuture {
            drivetrain,
            reverse: false,
            target_point: Vec2::from_array(point.map(|length| length.value)),
            timeout: self.timeout,
            tolerances: self.tolerances,
            linear_controller: self.linear_controller.clone(),
            lateral_controller: self.lateral_controller.clone(),
            state: None,
        }
    }
}
