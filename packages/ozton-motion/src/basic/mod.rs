//! Feedback-driven driving and turning.

use std::time::Duration;

use glam::DVec2 as Vec2;
use ozton_control::{Tolerances, axis::Linear, loops::Feedback};
use ozton_drivetrain::{Drivetrain, model::Arcade};
use ozton_math::{
    si,
    uom::si::f64::{Angle as UnitAngle, Length},
};
use ozton_tracking::{TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity};
use vexide::math::Angle;

mod drive;
mod turn_to_point;

pub use drive::DriveFuture;
pub use turn_to_point::TurnToPointFuture;

/// Feedback-driven driving and turning.
#[derive(PartialEq)]
pub struct Basic<L, A>
where
    L: Feedback<State = f64, Signal = f64> + Unpin + Clone,
    A: Feedback<State = Angle, Signal = f64> + Unpin + Clone,
{
    /// Linear (forward driving) feedback controller.
    pub linear_controller: L,

    /// Angular (turning) feedback controller.
    pub angular_controller: A,

    /// Linear settling conditions.
    pub linear_tolerances: Tolerances<Linear>,

    /// Angular settling conditions.
    pub angular_tolerances: Tolerances,

    /// Maximum duration the motion can take before being cancelled.
    pub timeout: Option<Duration>,
}

impl<L, A> Basic<L, A>
where
    L: Feedback<State = f64, Signal = f64> + Unpin + Clone,
    A: Feedback<State = Angle, Signal = f64> + Unpin + Clone,
{
    /// Moves the robot forwards by a given distance (measured in wheel units) while
    /// turning to face a heading.
    ///
    /// Negative `target_distance` values will move the robot backwards.
    pub fn drive_distance_at_heading<
        'a,
        M: Arcade,
        T: TracksForwardTravel + TracksHeading + TracksVelocity,
    >(
        &mut self,
        drivetrain: &'a mut Drivetrain<M, T>,
        target_distance: Length,
        target_heading: UnitAngle,
    ) -> DriveFuture<'a, M, L, A, T> {
        DriveFuture {
            target_distance: target_distance.value,
            target_heading: Angle::from_radians(target_heading.value),
            timeout: self.timeout,
            linear_tolerances: self.linear_tolerances,
            angular_tolerances: self.angular_tolerances,
            linear_controller: self.linear_controller.clone(),
            angular_controller: self.angular_controller.clone(),
            drivetrain,
            state: None,
        }
    }

    /// Moves the robot forwards by a given distance (measured in wheel units).
    ///
    /// Negative `distance` values will move the robot backwards.
    pub fn drive_distance<
        'a,
        M: Arcade,
        T: TracksForwardTravel + TracksHeading + TracksVelocity,
    >(
        &mut self,
        drivetrain: &'a mut Drivetrain<M, T>,
        distance: Length,
    ) -> DriveFuture<'a, M, L, A, T> {
        self.drive_distance_at_heading(
            drivetrain,
            distance,
            si(drivetrain.tracking.heading().as_radians()),
        )
    }

    /// Turns the robot in place to face a heading.
    pub fn turn_to_heading<
        'a,
        M: Arcade,
        T: TracksForwardTravel + TracksHeading + TracksVelocity,
    >(
        &mut self,
        drivetrain: &'a mut Drivetrain<M, T>,
        heading: UnitAngle,
    ) -> DriveFuture<'a, M, L, A, T> {
        self.drive_distance_at_heading(drivetrain, si(0.0), heading)
    }

    /// Turns the robot in place to face a 2D point.
    pub fn turn_to_point<
        'a,
        M: Arcade,
        T: TracksForwardTravel + TracksPosition + TracksHeading + TracksVelocity,
    >(
        &mut self,
        drivetrain: &'a mut Drivetrain<M, T>,
        point: [Length; 2],
    ) -> TurnToPointFuture<'a, M, L, A, T> {
        TurnToPointFuture {
            point: Vec2::from_array(point.map(|length| length.value)),
            timeout: self.timeout,
            linear_tolerances: self.linear_tolerances,
            angular_tolerances: self.angular_tolerances,
            linear_controller: self.linear_controller.clone(),
            angular_controller: self.angular_controller.clone(),
            drivetrain,
            state: None,
        }
    }
}
