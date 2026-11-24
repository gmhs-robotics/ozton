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

pub mod pose;
pub use pose::{DriveMotionFault, PoseDrive, PoseMotionError, TravelDirection};

/// One reusable seeking primitive. The default backend uses calibrated physical
/// wheel velocities and can seek either a point or a complete pose.
/// Use `Seeking::normalized()` for lightweight point-only voltage control
/// on a drivetrain that does not implement `PoseDrive`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Seeking<C = crate::MotionConfig> {
    config: C,
    travel: Option<TravelDirection>,
}

impl<C> Seeking<C> {
    /// Choose a direction rather than letting physical seeking select one.
    #[must_use]
    pub const fn with_travel(mut self, travel: TravelDirection) -> Self {
        self.travel = Some(travel);
        self
    }

    #[must_use]
    pub const fn reverse(self) -> Self {
        self.with_travel(TravelDirection::Backward)
    }

    #[must_use]
    pub const fn forward(self) -> Self {
        self.with_travel(TravelDirection::Forward)
    }

    #[must_use]
    pub const fn with_automatic_travel(mut self) -> Self {
        self.travel = None;
        self
    }
}

/// Settings for the opt-in normalized motor-output backend.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Normalized<L = Pid, A = Pid>
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

impl Seeking {
    /// Opt into the original normalized-output point-seeking backend.
    /// Its PID gains are motor fractions, not metres or radians per second.
    pub const fn normalized() -> Seeking<Normalized> {
        let linear = Pid::new(1.2, 0.0, 0.06, None).with_output_limit(0.65);
        let lateral = Pid::new(1.8, 0.0, 0.06, None).with_output_limit(0.55);
        Seeking::from_controllers(linear, lateral)
    }
}

impl Seeking<Normalized> {
    #[must_use]
    pub const fn with_linear_gains(mut self, kp: f64, ki: f64, kd: f64) -> Self {
        self.config.linear_controller.set_gains(kp, ki, kd);
        self
    }

    #[must_use]
    pub const fn with_linear_kp(mut self, kp: f64) -> Self {
        self.config.linear_controller.set_kp(kp);
        self
    }

    #[must_use]
    pub const fn with_linear_ki(mut self, ki: f64) -> Self {
        self.config.linear_controller.set_ki(ki);
        self
    }

    #[must_use]
    pub const fn with_linear_kd(mut self, kd: f64) -> Self {
        self.config.linear_controller.set_kd(kd);
        self
    }

    #[must_use]
    pub const fn with_linear_output_limit(mut self, limit: f64) -> Self {
        self.config.linear_controller.set_output_limit(Some(limit));
        self
    }

    #[must_use]
    pub const fn with_linear_integration_range(mut self, range: Length) -> Self {
        self.config.linear_controller
            .set_integration_range(Some(range.value));
        self
    }

    #[must_use]
    pub const fn without_linear_output_limit(mut self) -> Self {
        self.config.linear_controller.set_output_limit(None);
        self
    }

    #[must_use]
    pub const fn without_linear_integration_range(mut self) -> Self {
        self.config.linear_controller.set_integration_range(None);
        self
    }

    #[must_use]
    pub const fn with_lateral_gains(mut self, kp: f64, ki: f64, kd: f64) -> Self {
        self.config.lateral_controller.set_gains(kp, ki, kd);
        self
    }

    #[must_use]
    pub const fn with_lateral_kp(mut self, kp: f64) -> Self {
        self.config.lateral_controller.set_kp(kp);
        self
    }

    #[must_use]
    pub const fn with_lateral_ki(mut self, ki: f64) -> Self {
        self.config.lateral_controller.set_ki(ki);
        self
    }

    #[must_use]
    pub const fn with_lateral_kd(mut self, kd: f64) -> Self {
        self.config.lateral_controller.set_kd(kd);
        self
    }

    #[must_use]
    pub const fn with_lateral_output_limit(mut self, limit: f64) -> Self {
        self.config.lateral_controller.set_output_limit(Some(limit));
        self
    }

    #[must_use]
    pub const fn with_lateral_integration_range(mut self, range: Length) -> Self {
        self.config.lateral_controller
            .set_integration_range(Some(range.value));
        self
    }

    #[must_use]
    pub const fn without_lateral_output_limit(mut self) -> Self {
        self.config.lateral_controller.set_output_limit(None);
        self
    }

    #[must_use]
    pub const fn without_lateral_integration_range(mut self) -> Self {
        self.config.lateral_controller.set_integration_range(None);
        self
    }

    #[must_use]
    pub const fn with_angular_gains(self, kp: f64, ki: f64, kd: f64) -> Self {
        self.with_lateral_gains(kp, ki, kd)
    }
}

impl<L, A> Seeking<Normalized<L, A>>
where
    L: Feedback<State = f64, Signal = f64> + Unpin + Clone,
    A: Feedback<State = f64, Signal = f64> + Unpin + Clone,
{
    /// Inspect the normalized backend settings without modifying a template.
    pub const fn config(&self) -> &Normalized<L, A> {
        &self.config
    }

    /// Construct a template from custom feedback controllers without requiring PID.
    pub const fn from_controllers(linear_controller: L, lateral_controller: A) -> Self {
        Self {
            config: Normalized {
                linear_controller,
                lateral_controller,
                tolerances: crate::LINEAR_SETTLING,
                timeout: Some(crate::defaults::AUTON_STAGE_TIMEOUT),
            },
            travel: None,
        }
    }

    #[must_use]
    pub const fn with_timeout(mut self, timeout: Duration) -> Self {
        self.config.timeout = Some(timeout);
        self
    }

    #[must_use]
    pub const fn without_timeout(mut self) -> Self {
        self.config.timeout = None;
        self
    }

    #[must_use]
    pub const fn with_tolerances(mut self, tolerances: Tolerances<Linear>) -> Self {
        self.config.tolerances = tolerances;
        self
    }

    #[must_use]
    pub const fn with_error_tolerance(mut self, value: Length) -> Self {
        self.config.tolerances.error_tolerance = Some(value);
        self
    }

    #[must_use]
    pub const fn without_error_tolerance(mut self) -> Self {
        self.config.tolerances.error_tolerance = None;
        self
    }

    #[must_use]
    pub const fn with_velocity_tolerance(mut self, value: Velocity) -> Self {
        self.config.tolerances.velocity_tolerance = Some(value);
        self
    }

    #[must_use]
    pub const fn without_velocity_tolerance(mut self) -> Self {
        self.config.tolerances.velocity_tolerance = None;
        self
    }

    #[must_use]
    pub const fn with_tolerance_duration(mut self, value: Duration) -> Self {
        self.config.tolerances.duration = Some(value);
        self
    }

    #[must_use]
    pub const fn without_tolerance_duration(mut self) -> Self {
        self.config.tolerances.duration = None;
        self
    }

    #[must_use]
    pub const fn with_linear_controller<C>(self, controller: C) -> Seeking<Normalized<C, A>>
    where
        L: Copy,
        A: Copy,
        C: Feedback<State = f64, Signal = f64> + Unpin + Clone,
    {
        Seeking {
            config: Normalized {
                linear_controller: controller,
                lateral_controller: self.config.lateral_controller,
                tolerances: self.config.tolerances,
                timeout: self.config.timeout,
            },
            travel: self.travel,
        }
    }

    #[must_use]
    pub const fn with_lateral_controller<C>(self, controller: C) -> Seeking<Normalized<L, C>>
    where
        L: Copy,
        A: Copy,
        C: Feedback<State = f64, Signal = f64> + Unpin + Clone,
    {
        Seeking {
            config: Normalized {
                lateral_controller: controller,
                linear_controller: self.config.linear_controller,
                tolerances: self.config.tolerances,
                timeout: self.config.timeout,
            },
            travel: self.travel,
        }
    }
}

impl<L, A> Seeking<Normalized<L, A>>
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
            reverse: matches!(self.travel, Some(TravelDirection::Backward)),
            target_point: point.into(),
            timeout: self.config.timeout,
            tolerances: self.config.tolerances,
            linear_controller: self.config.linear_controller.clone(),
            lateral_controller: self.config.lateral_controller.clone(),
            state: None,
        }
    }
}

