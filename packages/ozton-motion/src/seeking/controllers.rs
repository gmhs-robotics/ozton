//! Standard motion capabilities, implemented only by controllers that support them.
use std::time::Duration;

use ozton_drivetrain::{MotionExecution, RangeResetDrive, WheelActuation};
use ozton_math::{
    geometry::{DVec2, Pose},
    path::{Path, PathSample, clearance::RectangularClearance},
    uom::si::f64::{Angle, Length, Velocity},
};

use super::{Engine, PathActions, PoseMotionError, TravelDirection};
use crate::{MotionConfig, MotionLimits, PathGuidance, SettlingConfig, TerminalGuidance};

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

    /// Follow straight legs with chained collinear transit points.
    pub trait FollowWaypoints {
        async fn follow_waypoints<D: MotionExecution>(
            &self,
            drive: &mut D,
            points: &[DVec2],
            handoff_radius: Length,
        ) -> MotionResult<D>;
    }

    /// Chain transit points where corner cutting is acceptable.
    pub trait MoveThroughPoints {
        async fn move_through_points<D: MotionExecution>(
            &self,
            drive: &mut D,
            points: &[DVec2],
            handoff_radius: Length,
        ) -> MotionResult<D>;
    }

    /// Follow a sequence of settled poses.
    pub trait FollowPoses {
        async fn follow_poses<D: MotionExecution>(
            &self,
            drive: &mut D,
            poses: &[Pose],
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
        async fn follow_path<D: MotionExecution>(
            &self,
            drive: &mut D,
            path: &Path,
        ) -> MotionResult<D> {
            self.follow_path_with_actions(drive, path, &mut PathActions::new())
                .await
        }

        async fn follow_path_with_actions<D: MotionExecution>(
            &self,
            drive: &mut D,
            path: &Path,
            actions: &mut PathActions<'_>,
        ) -> MotionResult<D>;
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
    PathGuidance::Tangent
);
controller!(
    /// Continuous Boomerang pose feedback.
    Boomerang,
    PathGuidance::Tangent
);
controller!(
    /// Spatial tangent trajectory tracking.
    Tangent,
    PathGuidance::Tangent
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
    /// Continuous-time speed-scheduled LQR trajectory tracking.
    LtvLqr,
    PathGuidance::LQR
);

impl DriveDistance for Basic {
    async fn drive_distance<D: MotionExecution>(
        &self,
        drive: &mut D,
        distance: Length,
    ) -> MotionResult<D> {
        self.engine.drive_distance(drive, distance).await
    }
}

impl TurnToHeading for Basic {
    async fn turn_to_heading<D: MotionExecution>(
        &self,
        drive: &mut D,
        heading: Angle,
    ) -> MotionResult<D> {
        self.engine.turn_to_heading(drive, heading).await
    }
}

impl TurnToPoint for Basic {
    async fn turn_to_point<D: MotionExecution>(
        &self,
        drive: &mut D,
        point: DVec2,
    ) -> MotionResult<D> {
        self.engine.turn_to_point(drive, point).await
    }
}

impl MoveToPoint for Basic {
    async fn move_to_point<D: MotionExecution>(
        &self,
        drive: &mut D,
        point: DVec2,
    ) -> MotionResult<D> {
        self.engine.move_to_point(drive, point).await
    }
}

impl MoveToPose for Basic {
    async fn move_to_pose<D: MotionExecution>(&self, drive: &mut D, pose: Pose) -> MotionResult<D> {
        self.engine.move_to_pose_straight_checked(drive, pose).await
    }
}

impl DriveArc for Basic {
    async fn drive_arc<D: MotionExecution>(
        &self,
        drive: &mut D,
        radius: Length,
        sweep: Angle,
    ) -> MotionResult<D> {
        self.engine.drive_arc(drive, radius, sweep).await
    }
}

impl FollowWaypoints for Basic {
    async fn follow_waypoints<D: MotionExecution>(
        &self,
        drive: &mut D,
        points: &[DVec2],
        handoff_radius: Length,
    ) -> MotionResult<D> {
        self.engine
            .move_through_points_straight(drive, points, handoff_radius)
            .await
    }
}

impl MoveThroughPoints for Basic {
    async fn move_through_points<D: MotionExecution>(
        &self,
        drive: &mut D,
        points: &[DVec2],
        handoff_radius: Length,
    ) -> MotionResult<D> {
        self.engine
            .move_through_points(drive, points, handoff_radius)
            .await
    }
}

impl FollowPoses for Basic {
    async fn follow_poses<D: MotionExecution>(
        &self,
        drive: &mut D,
        poses: &[Pose],
    ) -> MotionResult<D> {
        self.engine.follow_poses(drive, poses).await
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
        self.engine.move_to_pose(drive, pose).await
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
            async fn follow_path_with_actions<D: MotionExecution>(
                &self,
                drive: &mut D,
                path: &Path,
                actions: &mut PathActions<'_>,
            ) -> MotionResult<D> {
                self.engine
                    .follow_with_actions(
                        drive,
                        path,
                        self.engine.travel == Some(TravelDirection::Backward),
                        actions,
                    )
                    .await
            }
        }
    )+};
}

path_follower!(Tangent, PurePursuit, Ramsete, LtvLqr);

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

/// Build the configured physical clearance model without selecting a controller.
pub fn clearance<D: MotionExecution>(
    drive: &D,
    config: MotionConfig,
) -> MotionResult<D, Option<RectangularClearance>> {
    Engine {
        config,
        travel: None,
    }
    .clearance(drive)
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
