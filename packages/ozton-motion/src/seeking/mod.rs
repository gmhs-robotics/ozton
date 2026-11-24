//! Point-to-point feedback seeking.

use std::time::Duration;

use glam::DVec2 as Vec2;
use ozton_control::{
    Tolerances,
    axis::Linear,
    loops::{Feedback, Pid},
};
use ozton_drivetrain::{Drive, HasLocalization};
use ozton_math::{
    localization::observation::{TracksHeading, TracksPosition, TracksVelocity},
    uom::si::f64::{Length, Velocity},
};

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
pub struct Seeking<L = Pid, A = Pid>
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

impl Default for Seeking<Pid, Pid> {
    fn default() -> Self {
        Self::new()
    }
}

impl Seeking<Pid, Pid> {
    /// Reusable controller template with conservative defaults.
    pub const fn new() -> Self {
        let mut linear_controller = Pid::new(1.2, 0.0, 0.06, None);
        linear_controller.set_output_limit(Some(0.65));
        let mut lateral_controller = Pid::new(1.8, 0.0, 0.06, None);
        lateral_controller.set_output_limit(Some(0.55));
        Self {
            linear_controller,
            lateral_controller,
            tolerances: crate::LINEAR_SETTLING,
            timeout: Some(crate::defaults::AUTON_STAGE_TIMEOUT),
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
    pub const fn with_lateral_gains(mut self, kp: f64, ki: f64, kd: f64) -> Self {
        self.lateral_controller.set_gains(kp, ki, kd);
        self
    }

    #[must_use]
    pub const fn with_lateral_kp(mut self, kp: f64) -> Self {
        self.lateral_controller.set_kp(kp);
        self
    }

    #[must_use]
    pub const fn with_lateral_ki(mut self, ki: f64) -> Self {
        self.lateral_controller.set_ki(ki);
        self
    }

    #[must_use]
    pub const fn with_lateral_kd(mut self, kd: f64) -> Self {
        self.lateral_controller.set_kd(kd);
        self
    }

    #[must_use]
    pub const fn with_lateral_output_limit(mut self, limit: f64) -> Self {
        self.lateral_controller.set_output_limit(Some(limit));
        self
    }

    #[must_use]
    pub const fn with_lateral_integration_range(mut self, range: Length) -> Self {
        self.lateral_controller
            .set_integration_range(Some(range.value));
        self
    }

    #[must_use]
    pub const fn without_lateral_output_limit(mut self) -> Self {
        self.lateral_controller.set_output_limit(None);
        self
    }

    #[must_use]
    pub const fn without_lateral_integration_range(mut self) -> Self {
        self.lateral_controller.set_integration_range(None);
        self
    }

    #[must_use]
    pub const fn with_angular_gains(self, kp: f64, ki: f64, kd: f64) -> Self {
        self.with_lateral_gains(kp, ki, kd)
    }
}

impl<L, A> Seeking<L, A>
where
    L: Feedback<State = f64, Signal = f64> + Unpin + Clone,
    A: Feedback<State = f64, Signal = f64> + Unpin + Clone,
{
    /// Construct a template from custom feedback controllers without requiring PID.
    pub const fn from_controllers(linear_controller: L, lateral_controller: A) -> Self {
        Self {
            linear_controller,
            lateral_controller,
            tolerances: crate::LINEAR_SETTLING,
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
    pub const fn with_tolerances(mut self, tolerances: Tolerances<Linear>) -> Self {
        self.tolerances = tolerances;
        self
    }

    #[must_use]
    pub const fn with_error_tolerance(mut self, value: Length) -> Self {
        self.tolerances.error_tolerance = Some(value);
        self
    }

    #[must_use]
    pub const fn without_error_tolerance(mut self) -> Self {
        self.tolerances.error_tolerance = None;
        self
    }

    #[must_use]
    pub const fn with_velocity_tolerance(mut self, value: Velocity) -> Self {
        self.tolerances.velocity_tolerance = Some(value);
        self
    }

    #[must_use]
    pub const fn without_velocity_tolerance(mut self) -> Self {
        self.tolerances.velocity_tolerance = None;
        self
    }

    #[must_use]
    pub const fn with_tolerance_duration(mut self, value: Duration) -> Self {
        self.tolerances.duration = Some(value);
        self
    }

    #[must_use]
    pub const fn without_tolerance_duration(mut self) -> Self {
        self.tolerances.duration = None;
        self
    }

    #[must_use]
    pub const fn with_linear_controller<C>(self, controller: C) -> Seeking<C, A>
    where
        L: Copy,
        A: Copy,
        C: Feedback<State = f64, Signal = f64> + Unpin + Clone,
    {
        Seeking {
            linear_controller: controller,
            lateral_controller: self.lateral_controller,
            tolerances: self.tolerances,
            timeout: self.timeout,
        }
    }

    #[must_use]
    pub const fn with_lateral_controller<C>(self, controller: C) -> Seeking<L, C>
    where
        L: Copy,
        A: Copy,
        C: Feedback<State = f64, Signal = f64> + Unpin + Clone,
    {
        Seeking {
            lateral_controller: controller,
            linear_controller: self.linear_controller,
            tolerances: self.tolerances,
            timeout: self.timeout,
        }
    }
}

impl<L, A> Seeking<L, A>
where
    L: Feedback<State = f64, Signal = f64> + Unpin + Clone,
    A: Feedback<State = f64, Signal = f64> + Unpin + Clone,
{
    /// Moves the robot to a field point in metres, with coordinates right and forward.
    ///
    /// The final heading of the robot after this motion executes is undefined.
    pub fn move_to_point<
        'a,
        M: Drive + HasLocalization<Localization: TracksPosition + TracksHeading + TracksVelocity>,
    >(
        &self,
        drivetrain: &'a mut M,
        point: impl Into<Vec2>,
    ) -> MoveToPointFuture<'a, M, L, A> {
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
}

pub mod pose;
