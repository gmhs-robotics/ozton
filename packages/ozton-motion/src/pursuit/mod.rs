//! Adaptive pure pursuit controller.

extern crate alloc;

use alloc::vec::Vec;
use std::time::Duration;

use glam::DVec2 as Vec2;
use ozton_drivetrain::{Drivetrain, model::Tank};
use ozton_math::uom::si::f64::Length;
use ozton_tracking::{Tracking, TracksHeading, TracksPosition};

mod follow;
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

impl PurePursuit {
    /// Moves a drivetrain along a set of discrete waypoints using pure pursuit.
    pub fn follow<
        'a,
        M: Tank,
        I: Iterator<Item = Waypoint> + Unpin,
        S: Tracking,
        T: TracksPosition + TracksHeading,
    >(
        &self,
        drivetrain: &'a mut Drivetrain<M, S, T>,
        waypoints: impl IntoIterator<Item = Waypoint, IntoIter = I>,
    ) -> PurePursuitFuture<'a, M, S, T, I> {
        PurePursuitFuture {
            drivetrain,
            state: None,
            waypoints: waypoints.into_iter(),
            lookahead_distance: self.lookahead_distance.value,
            track_width: self.track_width.value,
            timeout: self.timeout,
        }
    }
}
