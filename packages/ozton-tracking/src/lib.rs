//! Robot localization and tracking.
//!
//! This crate provides traits and systems for tracking and localizing the position, orientation,
//! velocity, and forward travel of a mobile robot in a 2D environment. The goal is to measure
//! accurate data about the robot and provide it to motion algorithms.
//!
//! The traits provided in this crate allow for motion algorithms to operate generically over any
//! tracking system implementation, provided that the implementation measures the necessary data.
//!
//! Tracking systems may choose to implement the following traits for motion algorithms to use:
//!
//! - [`TracksPosition`], for tracking the robot's 2D position (odometry).
//! - [`TracksHeading`], for tracking the robot's absolute orientation (heading).
//! - [`TracksVelocity`], for tracking the robot's linear and angular velocity.
//! - [`TracksForwardTravel`], for tracking the robot's signed forward wheel travel.
//!
//! Below is an example of a motion algorithm function that is generic across all differential
//! drivetrains with tracking systems that measure a robot's forward wheel travel:
//!
//! ```ignore
//! async fn motion(drivetrain: &mut Drivetrain<Differential, impl TracksForwardTravel>) {
//!     // Motion goes here...
//! }
//! ```
//!
//! Additionally, a reference implementation of a tracking system that performs wheeled odometry is
//! provided by the [`wheeled`] module.
//!
//! # Units
//!
//! Use metres, radians, seconds and SI derivatives for new code. Historical wheeled
//! APIs retain the caller's wheel-diameter unit; use metres there for interoperability.

macro_rules! log {
    ($($arg:tt)*) => {
        #[cfg(debug_assertions)]
        {
            ::std::println!("[ozton-tracking] {}", ::core::format_args!($($arg)*));
        }
    };
}

pub mod gps;
pub mod localizer;
mod sensor;
pub mod wheeled;

use glam::DVec2 as Vec2;
pub use gps::{GpsTracking, GpsTrackingConfig};
pub use sensor::{Gyro, RotarySensor};
use vexide::math::Angle;

/// Marker trait for a tracking system.
pub trait Tracking {}

/// Explicit no-op tracking type for drivetrains that do not have odometry.
///
/// This keeps the drivetrain type intact while intentionally not implementing any of the motion
/// data traits. Any route-following or feedback motion API that requires tracking information will
/// therefore fail at compile time rather than silently behaving as if real odometry exists.
#[derive(Default, Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub struct NoTracking;

/// Backwards-friendly alias for [`NoTracking`].
pub type DummyTracking = NoTracking;

impl Tracking for NoTracking {}
impl Tracking for () {}

/// A tracking system that localizes a robot's 2D position.
///
/// This trait defines a system for tracking and reporting the position of a mobile robot in a
/// 2D environment. The process of localizing a mobile robot's position is also commonly known
/// as odometry.
///
/// # Coordinate System
///
/// Report The Origin's position in a fixed surveyed field frame with +X right and +Y forward.
/// Body-mounted points use the same axes with The Origin at (0, 0, 0) and +Z up.
/// Use metres for interoperability with localization, paths and autonomous commands.
pub trait TracksPosition: Tracking {
    /// Returns The Origin's field position: X right, Y forward, in metres.
    fn position(&self) -> Vec2;
}

/// A tracking system that tracks a robot's absolute orientation.
///
/// This trait defines a system for tracking and reporting the orientation (heading) of a mobile
/// robot in a 2D environment. The robot's orientation refers to its direction of travel relative
/// to a fixed reference frame.
///
/// # Angle System
///
/// Heading zero faces +Y forward. Positive yaw turns left (CCW about +Z).
/// A +90-degree heading faces -X; -90 degrees faces +X.
pub trait TracksHeading: Tracking {
    /// Returns the robot's orientation bounded from [0, 2π] radians.
    fn heading(&self) -> Angle;
}

/// A tracking system that tracks a robot's linear and angular velocity.
///
/// # Units
///
/// - Linear velocity is measured in *wheel units per second*. In this instance, *wheel units*
///   refer to whatever units the user measured their wheel diameter with.
///
/// - Angular velocity is measured in *radians per second*.
pub trait TracksVelocity: Tracking {
    /// Returns the robot's estimated linear velocity in wheel units per second.
    fn linear_velocity(&self) -> f64;

    /// Returns the robot's estimated angular velocity in radians per second.
    fn angular_velocity(&self) -> f64;
}

/// A tracking system that tracks a robot's signed forward wheel travel.
///
/// # Units
///
/// Units are expected to be returned in *wheel units* — the same units that were used to measure the
/// robot's wheel diameter.
pub trait TracksForwardTravel: Tracking {
    /// Returns the signed forward wheel travel of the robot in wheel units.
    ///
    /// Positive values indicate forward movement from the origin of tracking, while negative
    /// values indicate backwards movement from the origin.
    fn forward_travel(&self) -> f64;
}

pub mod mount;

/// Define sensor IDs, ordered mounts and named device wiring from one layout.
#[macro_export]
macro_rules! sensor_layout {
    ($vis:vis enum $id:ident, $devices:ident => $mounts:ident: $mount:ty {
        $($sensor:ident: $device:ident => $value:expr),+ $(,)?
    }) => {
        /// Sensor IDs in mount-array order.
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        #[repr(usize)]
        $vis enum $id { $($sensor),+ }

        $vis const $mounts: [$mount; [$(stringify!($sensor)),+].len()] = [$($value),+];

        /// One device for every mount in the layout.
        $vis struct $devices<T> {
            $(pub $device: T),+
        }

        impl<T> $devices<T> {
            /// Move the devices into an array in mount order.
            $vis fn into_array(self) -> [T; $mounts.len()] {
                [$(self.$device),+]
            }
        }
    };
}
