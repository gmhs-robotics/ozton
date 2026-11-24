//! Feedback-driven driving and turning.

use std::time::Duration;

use glam::DVec2 as Vec2;
use ozton_control::{
    Tolerances,
    axis::Linear,
    loops::{AngularPid, Feedback, Pid},
};
use ozton_drivetrain::{Drive, HasLocalization};
use ozton_math::uom::si::{
    angle::radian,
    f64::{Angle as UnitAngle, Length},
    length::meter,
};
use ozton_tracking::{TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity};
use vexide::math::Angle;

use crate::commands::routine_settings::{
    AUTON_ANGULAR_SPEED_TOLERANCE, AUTON_HEADING_TOLERANCE, AUTON_SETTLE_TIME, AUTON_STAGE_TIMEOUT,
};

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

impl Default for Basic<Pid, AngularPid> {
    fn default() -> Self {
        let mut linear_controller = Pid::new(1.2, 0.0, 0.06, None);
        linear_controller.set_output_limit(Some(0.65));
        let mut angular_controller = AngularPid::new(1.6, 0.0, 0.08, None);
        angular_controller.set_output_limit(Some(0.55));
        Self {
            linear_controller,
            angular_controller,
            linear_tolerances: crate::LINEAR_SETTLING,
            angular_tolerances: Tolerances::new()
                .error(AUTON_HEADING_TOLERANCE)
                .velocity(AUTON_ANGULAR_SPEED_TOLERANCE)
                .duration(AUTON_SETTLE_TIME),
            timeout: Some(AUTON_STAGE_TIMEOUT),
        }
    }
}

impl<L, A> Basic<L, A>
where
    L: Feedback<State = f64, Signal = f64> + Unpin + Clone,
    A: Feedback<State = Angle, Signal = f64> + Unpin + Clone,
{
    /// Moves the robot forwards by a given distance while turning to face a heading.
    ///
    /// Negative `target_distance` values will move the robot backwards.
    pub fn drive_distance_at_heading<
        'a,
        M: Drive + HasLocalization<Localization: TracksForwardTravel + TracksHeading + TracksVelocity>,
    >(
        &self,
        drivetrain: &'a mut M,
        target_distance: Length,
        target_heading: UnitAngle,
    ) -> DriveFuture<'a, M, L, A> {
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

    /// Moves the robot forwards by a given distance.
    ///
    /// Negative `distance` values will move the robot backwards.
    pub fn drive_distance<
        'a,
        M: Drive + HasLocalization<Localization: TracksForwardTravel + TracksHeading + TracksVelocity>,
    >(
        &self,
        drivetrain: &'a mut M,
        distance: Length,
    ) -> DriveFuture<'a, M, L, A> {
        drivetrain.refresh_localization();
        self.drive_distance_at_heading(
            drivetrain,
            distance,
            UnitAngle::new::<radian>(drivetrain.localization().heading().as_radians()),
        )
    }

    /// Turns the robot in place to face a heading.
    pub fn turn_to_heading<
        'a,
        M: Drive + HasLocalization<Localization: TracksForwardTravel + TracksHeading + TracksVelocity>,
    >(
        &self,
        drivetrain: &'a mut M,
        heading: UnitAngle,
    ) -> DriveFuture<'a, M, L, A> {
        self.drive_distance_at_heading(drivetrain, Length::new::<meter>(0.0), heading)
    }

    /// Turns the robot in place to face a 2D point.
    pub fn turn_to_point<
        'a,
        M: Drive
            + HasLocalization<
                Localization: TracksForwardTravel + TracksPosition + TracksHeading + TracksVelocity,
            >,
    >(
        &self,
        drivetrain: &'a mut M,
        point: impl Into<Vec2>,
    ) -> TurnToPointFuture<'a, M, L, A> {
        TurnToPointFuture {
            point: point.into(),
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
