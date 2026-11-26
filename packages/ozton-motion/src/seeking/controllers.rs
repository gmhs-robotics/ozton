//! Standard motion capabilities, implemented only by controllers that support them.
use std::time::Duration;

use ozton_drivetrain::{MotionExecution, RangeResetDrive, WheelActuation};
use ozton_math::{
    geometry::{DVec2, Pose},
    path::{Path, PathSample},
    uom::si::f64::{Angle, Length, Velocity},
};

use super::{Engine, MotionSegment, PoseMotionError, RunMotion, TravelDirection, pose::Command};
use crate::{
    DriveTuning, MotionConfig, MotionLimits, PathGuidance, SettlingConfig, TerminalGuidance,
};

pub type MotionResult<D, T = ()> = Result<T, PoseMotionError<<D as MotionExecution>::Error>>;

/// A controller implements only the commands it supports. The drivetrain
/// is borrowed for each command, so different controllers can use the same
/// localization estimate without binding or owning the hardware.
#[allow(async_fn_in_trait)]
pub mod capabilities {
    use super::*;

    /// Signed distance, preserving heading.
    pub trait DriveDistance {
        async fn drive_distance<D: MotionExecution>(
            &self,
            drive: &mut D,
            distance: Length,
        ) -> MotionResult<D>;
    }

    /// Absolute heading turn.
    pub trait TurnToHeading {
        async fn turn_to_heading<D: MotionExecution>(
            &self,
            drive: &mut D,
            heading: Angle,
        ) -> MotionResult<D>;
    }

    /// Turn toward a field position.
    pub trait TurnToPoint {
        async fn turn_to_point<D: MotionExecution>(
            &self,
            drive: &mut D,
            point: DVec2,
        ) -> MotionResult<D>;
    }

    /// Move to a field position, final heading free.
    pub trait MoveToPoint {
        async fn move_to_point<D: MotionExecution>(
            &self,
            drive: &mut D,
            point: DVec2,
        ) -> MotionResult<D>;
    }

    /// Move to an absolute field pose.
    pub trait MoveToPose {
        async fn move_to_pose<D: MotionExecution>(
            &self,
            drive: &mut D,
            pose: Pose,
        ) -> MotionResult<D>;
    }

    /// Drive a constant-radius arc with signed sweep.
    pub trait DriveArc {
        async fn drive_arc<D: MotionExecution>(
            &self,
            drive: &mut D,
            radius: Length,
            sweep: Angle,
        ) -> MotionResult<D>;
    }

    /// Swing to a heading with one wheel group held.
    pub trait SwingToHeading {
        async fn swing_to_heading<D: MotionExecution + WheelActuation>(
            &self,
            drive: &mut D,
            heading: Angle,
        ) -> MotionResult<D>
        where
            D::Error: From<D::WheelError>;
    }

    /// Follow a prepared trajectory using this controller's tracking law.
    pub trait FollowPath {
        type Command<'a>: RunMotion + 'a;

        fn follow<'a>(&self, path: &'a Path) -> Self::Command<'a>;
    }
}

use capabilities::*;

// All templates share physical limits and feedback timing, not capabilities.
// The selected tracking law survives configuration replacement.
macro_rules! controller {
    ($(#[$doc:meta])* $name:ident, $guidance:expr) => {
        $(#[$doc])*
        #[derive(Clone, Copy, Debug, PartialEq)]
        pub struct $name {
            engine: Engine,
        }

        impl Default for $name {
            fn default() -> Self {
                Self::new()
            }
        }

        impl $name {
            pub const fn new() -> Self {
                let mut config = MotionConfig::DEFAULT;
                config.path.guidance = $guidance;
                Self {
                    engine: Engine { config, travel: None },
                }
            }

            pub const fn config(self) -> MotionConfig {
                self.engine.config
            }

            #[must_use]
            pub const fn with_config(mut self, mut config: MotionConfig) -> Self {
                config.path.guidance = self.engine.config.path.guidance;
                self.engine.config = config;
                self
            }

            #[must_use]
            pub const fn with_limits(mut self, limits: MotionLimits) -> Self {
                self.engine.config.limits = limits;
                self
            }

            #[must_use]
            pub const fn with_tuning(mut self, tuning: DriveTuning) -> Self {
                self.engine.config.tuning = tuning;
                self
            }

            #[must_use]
            pub const fn with_settling(mut self, settling: SettlingConfig) -> Self {
                self.engine.config.settling = settling;
                self
            }

            #[must_use]
            pub const fn with_wheel_speed(mut self, speed: Velocity) -> Self {
                self.engine.config.limits = self.engine.config.limits.with_wheel_speed(speed);
                self
            }

            #[must_use]
            pub const fn with_timeout(mut self, timeout: Duration) -> Self {
                self.engine.config.timing = self.engine.config.timing.with_stage_timeout(timeout);
                self
            }

            #[must_use]
            pub const fn with_travel(mut self, travel: TravelDirection) -> Self {
                self.engine.travel = Some(travel);
                self
            }
        }
    };
}

controller!(
    /// Conventional distance, point, heading and straight-pose PID.
    Basic,
    PathGuidance::PurePursuit
);
controller!(
    /// Continuous Boomerang pose feedback.
    Boomerang,
    PathGuidance::PurePursuit
);
controller!(
    /// Geometric lookahead trajectory tracking.
    PurePursuit,
    PathGuidance::PurePursuit
);
controller!(
    /// Nonlinear RAMSETE trajectory tracking.
    Ramsete,
    PathGuidance::RAMSETE
);
controller!(
    /// Finite-horizon discrete LTV-LQR trajectory tracking.
    LtvLqr,
    PathGuidance::LQR
);

impl Basic {
    pub fn distance(self, distance: Length) -> MotionSegment<'static> {
        MotionSegment::new(self.engine, Command::Distance(distance))
    }

    pub fn to_heading(self, heading: Angle) -> MotionSegment<'static> {
        MotionSegment::new(self.engine, Command::Heading(heading))
    }

    pub fn toward(self, point: DVec2) -> MotionSegment<'static> {
        MotionSegment::new(self.engine, Command::TurnPoint(point))
    }

    pub fn to_point(self, point: DVec2) -> MotionSegment<'static> {
        MotionSegment::new(self.engine, Command::Point(point))
    }

    pub fn to_pose(self, pose: Pose) -> MotionSegment<'static> {
        MotionSegment::new(self.engine, Command::StraightPose(pose))
    }

    pub fn arc(self, radius: Length, sweep: Angle) -> MotionSegment<'static> {
        MotionSegment::new(self.engine, Command::Arc(radius, sweep))
    }
}

impl DriveDistance for Basic {
    async fn drive_distance<D: MotionExecution>(
        &self,
        drive: &mut D,
        distance: Length,
    ) -> MotionResult<D> {
        self.distance(distance).run(drive).await
    }
}

impl TurnToHeading for Basic {
    async fn turn_to_heading<D: MotionExecution>(
        &self,
        drive: &mut D,
        heading: Angle,
    ) -> MotionResult<D> {
        self.to_heading(heading).run(drive).await
    }
}

impl TurnToPoint for Basic {
    async fn turn_to_point<D: MotionExecution>(
        &self,
        drive: &mut D,
        point: DVec2,
    ) -> MotionResult<D> {
        self.toward(point).run(drive).await
    }
}

impl MoveToPoint for Basic {
    async fn move_to_point<D: MotionExecution>(
        &self,
        drive: &mut D,
        point: DVec2,
    ) -> MotionResult<D> {
        self.to_point(point).run(drive).await
    }
}

impl MoveToPose for Basic {
    async fn move_to_pose<D: MotionExecution>(&self, drive: &mut D, pose: Pose) -> MotionResult<D> {
        self.to_pose(pose).run(drive).await
    }
}

impl DriveArc for Basic {
    async fn drive_arc<D: MotionExecution>(
        &self,
        drive: &mut D,
        radius: Length,
        sweep: Angle,
    ) -> MotionResult<D> {
        self.arc(radius, sweep).run(drive).await
    }
}

impl SwingToHeading for Basic {
    async fn swing_to_heading<D: MotionExecution + WheelActuation>(
        &self,
        drive: &mut D,
        heading: Angle,
    ) -> MotionResult<D>
    where
        D::Error: From<D::WheelError>,
    {
        self.engine.swing_to_heading(drive, heading).await
    }
}

impl MoveToPose for Boomerang {
    async fn move_to_pose<D: MotionExecution>(&self, drive: &mut D, pose: Pose) -> MotionResult<D> {
        self.to_pose(pose).run(drive).await
    }
}

impl PurePursuit {
    /// Fixed lookahead radius. Classical pursuit always needs a target radius.
    #[must_use]
    pub const fn with_lookahead(mut self, radius: Length) -> Self {
        self.engine.config.path = self.engine.config.path.with_lookahead(radius);
        self
    }

    /// Optional velocity-scaled lookahead, bounded below by the fixed radius.
    #[must_use]
    pub const fn with_adaptive_lookahead(mut self, horizon: Duration) -> Self {
        self.engine.config.path = self.engine.config.path.with_adaptive_lookahead(horizon);
        self
    }

    #[must_use]
    pub const fn without_adaptive_lookahead(mut self) -> Self {
        self.engine.config.path = self.engine.config.path.without_adaptive_lookahead();
        self
    }

    /// Optional constant-twist pose and velocity-profile prediction.
    #[must_use]
    pub const fn with_prediction_latency(mut self, forward: Duration, yaw: Duration) -> Self {
        self.engine.config.path = self
            .engine
            .config
            .path
            .with_prediction_latency(forward, yaw);
        self
    }

    #[must_use]
    pub const fn without_prediction(mut self) -> Self {
        self.engine.config.path = self.engine.config.path.without_prediction();
        self
    }

    /// Optional PID alignment when the pursuit target is behind the robot.
    #[must_use]
    pub const fn with_heading_recovery(mut self, enabled: bool) -> Self {
        self.engine.config.path = self.engine.config.path.with_heading_recovery(enabled);
        self
    }
}

impl Boomerang {
    pub fn to_pose(self, pose: Pose) -> MotionSegment<'static> {
        MotionSegment::new(self.engine, Command::Pose(pose))
    }

    /// Fraction of remaining distance projected behind the goal.
    #[must_use]
    pub const fn with_lead(mut self, lead: f64) -> Self {
        self.engine.config.approach = self.engine.config.approach.with_boomerang_lead(lead);
        self
    }
}

impl Ramsete {
    /// Set the nonlinear feedback gains. Invalid gains fail motion validation.
    #[must_use]
    pub const fn with_gains(mut self, b: f64, zeta: f64) -> Self {
        self.engine.config.path.guidance = PathGuidance::Ramsete { b, zeta };
        self
    }
}

impl LtvLqr {
    /// Set state and input cost weights. Invalid weights fail motion validation.
    #[must_use]
    pub const fn with_weights(mut self, state: [f64; 3], input: [f64; 2]) -> Self {
        self.engine.config.path.guidance = PathGuidance::Lqr { state, input };
        self
    }
}

macro_rules! path_follower {
    ($($name:ident),+ $(,)?) => {$(
        impl $name {
            /// Bind a prepared trajectory to this tracking law.
            pub fn follow(self, path: &Path) -> MotionSegment<'_> {
                MotionSegment::new(self.engine, Command::Path(path))
            }

            /// Run a single trajectory without constructing a chain.
            pub async fn run<D: MotionExecution>(self, drive: &mut D, path: &Path) -> MotionResult<D> {
                self.follow(path).run(drive).await
            }

            /// Select the endpoint pose controller independently of tracking.
            #[must_use]
            pub const fn with_terminal(mut self, terminal: TerminalGuidance) -> Self {
                self.engine.config.path = self.engine.config.path.with_terminal(terminal);
                self
            }

            /// Use polar-coordinate feedback during terminal pose acquisition.
            #[must_use]
            pub const fn with_polar_terminal(
                mut self,
                k_rho: f64,
                k_alpha: f64,
                k_beta: f64,
            ) -> Self {
                self.engine.config.path.terminal = TerminalGuidance::Polar {
                    k_rho,
                    k_alpha,
                    k_beta,
                };
                self
            }
        }

        impl FollowPath for $name {
            type Command<'a> = MotionSegment<'a>;

            fn follow<'a>(&self, path: &'a Path) -> Self::Command<'a> {
                (*self).follow(path)
            }
        }
    )+};
}

path_follower!(PurePursuit, Ramsete, LtvLqr);

/// Validate host-generated path samples against the configured physical limits.
pub fn prepare_path<D: MotionExecution>(
    drive: &D,
    config: MotionConfig,
    samples: &'static [PathSample],
    limits: [f64; 7],
) -> MotionResult<D, Path> {
    Engine {
        config,
        travel: None,
    }
    .prepare_path(drive, samples, limits)
}

/// Hold the drivetrain stationary while the localization estimate settles.
pub async fn settle<D: MotionExecution>(
    drive: &mut D,
    config: MotionConfig,
    duration: Duration,
) -> MotionResult<D> {
    Engine {
        config,
        travel: None,
    }
    .settle(drive, duration)
    .await
}

/// Request a stationary distance-sensor localization correction.
pub async fn try_distance_reset<D: RangeResetDrive>(
    drive: &mut D,
    config: MotionConfig,
    timeout: Duration,
) -> MotionResult<D, bool> {
    Engine {
        config,
        travel: None,
    }
    .try_distance_reset(drive, timeout)
    .await
}
