//! Adaptive pure pursuit controller.

extern crate alloc;

use alloc::vec::Vec;
use std::time::Duration;

use glam::DVec2 as Vec2;
#[cfg(feature = "vex")]
use ozton_drivetrain::{Drive, HasLocalization};
#[cfg(feature = "vex")]
use ozton_math::localization::observation::{TracksHeading, TracksPosition};
use ozton_math::{control::Pid, uom::si::f64::Length};

/// Normalized-output endpoint controller for the Evian-compatible pursuit API.
/// Gains output motor fractions, not physical wheel velocities.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PursuitTerminalConfig {
    pub linear: Pid,
    pub angular: Pid,
    pub position_tolerance: Length,
    pub velocity_tolerance: f64,
    pub angular_velocity_tolerance: f64,
    pub settle: Duration,
    pub rate_filter: Duration,
    pub period: Duration,
}

impl Default for PursuitTerminalConfig {
    fn default() -> Self {
        let mut linear = Pid::new(2.0, 0.0, 0.10, None);
        let mut angular = Pid::new(1.5, 0.0, 0.06, None);
        linear.set_output_limit(Some(0.35));
        angular.set_output_limit(Some(0.40));
        Self {
            linear,
            angular,
            position_tolerance: crate::defaults::AUTON_POSITION_TOLERANCE,
            velocity_tolerance: 0.04,
            angular_velocity_tolerance: 0.12,
            settle: Duration::from_millis(120),
            rate_filter: Duration::from_millis(40),
            period: Duration::from_millis(10),
        }
    }
}

impl PursuitTerminalConfig {
    pub fn valid(self) -> bool {
        crate::DriveTuning {
            linear: self.linear,
            angular: self.angular,
        }
        .valid()
            && [
                self.position_tolerance.value,
                self.velocity_tolerance,
                self.angular_velocity_tolerance,
            ]
            .into_iter()
            .all(|value| value.is_finite() && value > 0.0)
            && !self.period.is_zero()
    }
}

#[cfg(feature = "vex")]
mod follow;
#[cfg(feature = "vex")]
pub use follow::PurePursuitFuture;

/// Parses a LemLib v0.5 path into positions in meters and normalized speed.
pub fn parse_lemlib_path(
    bytes: &[u8],
) -> Result<Vec<Waypoint>, ozton_path_io::jerryio::ImportError> {
    ozton_path_io::jerryio::parse(bytes).map(|points| {
        points
            .into_iter()
            .map(|point| Waypoint {
                position: point.position,
                speed_fraction: point.speed_fraction,
            })
            .collect()
    })
}

/// Pure pursuit path waypoint.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct Waypoint {
    /// Position of the robot at this point along the path.
    pub position: Vec2,

    /// Normalized forward output at this point along the path, from zero to one.
    pub speed_fraction: f64,
}

/// Adaptive pure pursuit controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PurePursuit {
    /// Radius of the robot's lookahead circle.
    pub lookahead_distance: Length,

    /// Distance between the left and right wheels on the robot.
    pub track_width: Length,

    /// Maximum duration the motion can take before being cancelled.
    pub timeout: Option<Duration>,
}

#[cfg(feature = "vex")]
impl PurePursuit {
    /// Moves a drivetrain along a set of discrete waypoints using pure pursuit.
    pub fn follow<
        'a,
        I: Iterator<Item = Waypoint> + Unpin,
        M: Drive + HasLocalization<Localization: TracksPosition + TracksHeading>,
    >(
        &self,
        drivetrain: &'a mut M,
        waypoints: impl IntoIterator<Item = Waypoint, IntoIter = I>,
    ) -> PurePursuitFuture<'a, M, I> {
        PurePursuitFuture {
            drivetrain,
            state: None,
            waypoints: waypoints.into_iter(),
            lookahead_distance: self.lookahead_distance.value,
            track_width: self.track_width.value,
            timeout: self.timeout,
            terminal_config: PursuitTerminalConfig::default(),
        }
    }
}

pub mod profiled;
