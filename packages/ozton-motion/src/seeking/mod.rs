//! Point-to-point feedback seeking.

use std::time::Duration;

use glam::DVec2 as Vec2;
use ozton_control::{Tolerances, loops::Feedback};
use ozton_drivetrain::{Drivetrain, model::Arcade};
use ozton_tracking::{TracksHeading, TracksPosition, TracksVelocity};

mod boomerang;
mod move_to_point;

pub use boomerang::BoomerangFuture;
pub use move_to_point::MoveToPointFuture;

/// Point-to-point feedback seeking.
///
/// This struct provides implementations of adaptive feedback seeking algorithms, which
/// utilize two feedback controllers (one for straight driving and one for turning) to
/// reach a desired point. This is most commonly done using two PID controllers.
///
/// Seeking motions include:
/// - [`move_to_point`](Seeking::move_to_point), which moves the drivetrain to a desired point.
/// - [`boomerang`](Seeking::move_to_point), which moves the drivetrain to a desired pose (including heading).
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
    pub tolerances: Tolerances,

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
    /// For full pose control, use [`Seeking::boomerang`].
    pub fn move_to_point<'a, M: Arcade, T: TracksPosition + TracksHeading + TracksVelocity>(
        &mut self,
        drivetrain: &'a mut Drivetrain<M, T>,
        point: impl Into<Vec2>,
    ) -> MoveToPointFuture<'a, M, L, A, T> {
        MoveToPointFuture {
            drivetrain,
            reverse: false,
            target_point: point.into(),
            timeout: self.timeout,
            tolerances: self.tolerances,
            linear_controller: self.linear_controller.clone(),
            lateral_controller: self.lateral_controller.clone(),
            state: None,
        }
    }

    // /// Moves the robot to a desired pose (position and heading).
    // ///
    // /// This motion uses a boomerang controller, which is a motion algorithm
    // /// for moving differential drivetrains to a desired pose. Larger `lead`
    // /// values will result in wider arcs, while smaller `lead` values will
    // /// result in smaller arcs. You may need to tune the `lead` value in order
    // /// to properly reach the desired heading by the end of the motion.
    // pub fn boomerang<'a, M: Arcade, T: TracksPosition + TracksHeading + TracksVelocity>(
    //     &mut self,
    //     drivetrain: &'a mut Drivetrain<M, T>,
    //     point: impl Into<Vec2>,
    //     heading: Angle,
    //     lead: f64,
    // ) -> BoomerangFuture<'a, M, L, A, T> {
    //     BoomerangFuture {
    //         drivetrain,
    //         target_heading: heading,
    //         lead,
    //         target_point: point.into(),
    //         timeout: self.timeout,
    //         tolerances: self.tolerances,
    //         linear_controller: self.linear_controller.clone(),
    //         angular_controller: self.angular_controller.clone(),
    //         state: None,
    //     }
    // }
}
