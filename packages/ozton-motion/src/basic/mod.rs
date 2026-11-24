//! Feedback-driven driving and turning.

use std::time::Duration;

#[cfg(feature = "vex")]
use glam::DVec2 as Vec2;
use ozton_control::{
    Tolerances,
    axis::Linear,
    loops::{AngularPid, Feedback, Pid},
};
#[cfg(feature = "vex")]
use ozton_drivetrain::{Drive, HasLocalization};
#[cfg(feature = "vex")]
use ozton_math::localization::observation::{
    TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity,
};
use ozton_math::uom::si::f64::{Angle, AngularVelocity, Length, Velocity};

use crate::defaults::{
    AUTON_ANGULAR_SPEED_TOLERANCE, AUTON_HEADING_TOLERANCE, AUTON_SETTLE_TIME, AUTON_STAGE_TIMEOUT,
};

#[cfg(feature = "vex")]
mod drive;
#[cfg(feature = "vex")]
mod turn_to_point;

#[cfg(feature = "vex")]
pub use drive::DriveFuture;
#[cfg(feature = "vex")]
pub use turn_to_point::TurnToPointFuture;

/// Feedback-driven driving and turning.
#[derive(PartialEq)]
pub struct Basic<L = Pid, A = AngularPid>
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
        Self::new()
    }
}

impl Basic<Pid, AngularPid> {
    /// Reusable controller template with conservative defaults.
    pub const fn new() -> Self {
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

    #[must_use]
    pub const fn with_linear_gains(mut self, kp: f64, ki: f64, kd: f64) -> Self {
        self.linear_controller.set_gains(kp, ki, kd);
        self
    }

    #[must_use]
    pub const fn with_linear_kp(mut self, kp: f64) -> Self {
        self.linear_controller.set_kp(kp);
        self
    }

    #[must_use]
    pub const fn with_linear_ki(mut self, ki: f64) -> Self {
        self.linear_controller.set_ki(ki);
        self
    }

    #[must_use]
    pub const fn with_linear_kd(mut self, kd: f64) -> Self {
        self.linear_controller.set_kd(kd);
        self
    }

    #[must_use]
    pub const fn with_linear_output_limit(mut self, limit: f64) -> Self {
        self.linear_controller.set_output_limit(Some(limit));
        self
    }

    #[must_use]
    pub const fn with_linear_integration_range(mut self, range: Length) -> Self {
        self.linear_controller
            .set_integration_range(Some(range.value));
        self
    }

    #[must_use]
    pub const fn without_linear_output_limit(mut self) -> Self {
        self.linear_controller.set_output_limit(None);
        self
    }

    #[must_use]
    pub const fn without_linear_integration_range(mut self) -> Self {
        self.linear_controller.set_integration_range(None);
        self
    }

    #[must_use]
    pub const fn with_angular_gains(mut self, kp: f64, ki: f64, kd: f64) -> Self {
        self.angular_controller.set_gains(kp, ki, kd);
        self
    }

    #[must_use]
    pub const fn with_angular_kp(mut self, kp: f64) -> Self {
        self.angular_controller.set_kp(kp);
        self
    }

    #[must_use]
    pub const fn with_angular_ki(mut self, ki: f64) -> Self {
        self.angular_controller.set_ki(ki);
        self
    }

    #[must_use]
    pub const fn with_angular_kd(mut self, kd: f64) -> Self {
        self.angular_controller.set_kd(kd);
        self
    }

    #[must_use]
    pub const fn with_angular_output_limit(mut self, limit: f64) -> Self {
        self.angular_controller.set_output_limit(Some(limit));
        self
    }

    #[must_use]
    pub const fn with_angular_integration_range(mut self, range: Angle) -> Self {
        self.angular_controller.set_integration_range(Some(range));
        self
    }

    #[must_use]
    pub const fn without_angular_output_limit(mut self) -> Self {
        self.angular_controller.set_output_limit(None);
        self
    }

    #[must_use]
    pub const fn without_angular_integration_range(mut self) -> Self {
        self.angular_controller.set_integration_range(None);
        self
    }
}

impl<L, A> Basic<L, A>
where
    L: Feedback<State = f64, Signal = f64> + Unpin + Clone,
    A: Feedback<State = Angle, Signal = f64> + Unpin + Clone,
{
    /// Construct a template from custom feedback controllers without requiring PID.
    pub const fn from_controllers(linear_controller: L, angular_controller: A) -> Self {
        Self {
            linear_controller,
            angular_controller,
            linear_tolerances: crate::LINEAR_SETTLING,
            angular_tolerances: Tolerances::new()
                .error(AUTON_HEADING_TOLERANCE)
                .velocity(AUTON_ANGULAR_SPEED_TOLERANCE)
                .duration(AUTON_SETTLE_TIME),
            timeout: Some(crate::defaults::AUTON_STAGE_TIMEOUT),
        }
    }

    #[must_use]
    pub const fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = Some(timeout);
        self
    }

    #[must_use]
    pub const fn without_timeout(mut self) -> Self {
        self.timeout = None;
        self
    }

    #[must_use]
    pub const fn with_linear_tolerances(mut self, tolerances: Tolerances<Linear>) -> Self {
        self.linear_tolerances = tolerances;
        self
    }

    #[must_use]
    pub const fn with_linear_error_tolerance(mut self, value: Length) -> Self {
        self.linear_tolerances.error_tolerance = Some(value);
        self
    }

    #[must_use]
    pub const fn without_linear_error_tolerance(mut self) -> Self {
        self.linear_tolerances.error_tolerance = None;
        self
    }

    #[must_use]
    pub const fn with_linear_velocity_tolerance(mut self, value: Velocity) -> Self {
        self.linear_tolerances.velocity_tolerance = Some(value);
        self
    }

    #[must_use]
    pub const fn without_linear_velocity_tolerance(mut self) -> Self {
        self.linear_tolerances.velocity_tolerance = None;
        self
    }

    #[must_use]
    pub const fn with_linear_tolerance_duration(mut self, value: Duration) -> Self {
        self.linear_tolerances.duration = Some(value);
        self
    }

    #[must_use]
    pub const fn without_linear_tolerance_duration(mut self) -> Self {
        self.linear_tolerances.duration = None;
        self
    }

    #[must_use]
    pub const fn with_angular_tolerances(mut self, tolerances: Tolerances) -> Self {
        self.angular_tolerances = tolerances;
        self
    }

    #[must_use]
    pub const fn with_angular_error_tolerance(mut self, value: Angle) -> Self {
        self.angular_tolerances.error_tolerance = Some(value);
        self
    }

    #[must_use]
    pub const fn without_angular_error_tolerance(mut self) -> Self {
        self.angular_tolerances.error_tolerance = None;
        self
    }

    #[must_use]
    pub const fn with_angular_velocity_tolerance(mut self, value: AngularVelocity) -> Self {
        self.angular_tolerances.velocity_tolerance = Some(value);
        self
    }

    #[must_use]
    pub const fn without_angular_velocity_tolerance(mut self) -> Self {
        self.angular_tolerances.velocity_tolerance = None;
        self
    }

    #[must_use]
    pub const fn with_angular_tolerance_duration(mut self, value: Duration) -> Self {
        self.angular_tolerances.duration = Some(value);
        self
    }

    #[must_use]
    pub const fn without_angular_tolerance_duration(mut self) -> Self {
        self.angular_tolerances.duration = None;
        self
    }

    #[must_use]
    pub const fn with_linear_controller<C>(self, controller: C) -> Basic<C, A>
    where
        L: Copy,
        A: Copy,
        C: Feedback<State = f64, Signal = f64> + Unpin + Clone,
    {
        Basic {
            linear_controller: controller,
            angular_controller: self.angular_controller,
            linear_tolerances: self.linear_tolerances,
            angular_tolerances: self.angular_tolerances,
            timeout: self.timeout,
        }
    }

    #[must_use]
    pub const fn with_angular_controller<C>(self, controller: C) -> Basic<L, C>
    where
        L: Copy,
        A: Copy,
        C: Feedback<State = Angle, Signal = f64> + Unpin + Clone,
    {
        Basic {
            angular_controller: controller,
            linear_controller: self.linear_controller,
            linear_tolerances: self.linear_tolerances,
            angular_tolerances: self.angular_tolerances,
            timeout: self.timeout,
        }
    }
}

#[cfg(feature = "vex")]
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
        target_heading: Angle,
    ) -> DriveFuture<'a, M, L, A> {
        DriveFuture {
            target_distance: target_distance.value,
            target_heading,
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
        let heading = drivetrain.localization().heading();
        self.drive_distance_at_heading(drivetrain, distance, heading)
    }

    /// Turns the robot in place to face a heading.
    pub fn turn_to_heading<
        'a,
        M: Drive + HasLocalization<Localization: TracksForwardTravel + TracksHeading + TracksVelocity>,
    >(
        &self,
        drivetrain: &'a mut M,
        heading: Angle,
    ) -> DriveFuture<'a, M, L, A> {
        self.drive_distance_at_heading(drivetrain, ozton_math::qty!(0.0 m), heading)
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

pub mod differential;
