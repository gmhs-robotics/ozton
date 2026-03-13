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
//! # A quick note about units!
//!
//! `ozton` made the intentional choice to be primarily unitless, mainly because stable Rust
//! currently lacks many of the features needed to provide a good developer experience using
//! typed units. Although several widely-used libraries exist in stable Rust (such as
//! [`uom`](http://crates.io/crates/uom)), there is no single obvious choice to be made here and
//! this region of the Rust ecosystem seems to be in a fairly volatile state that would make it
//! dangerous to fully commit to a single library this early on.
//!
//! As such, ozton made the decision to forego requiring specific units of measure where possible,
//! particular in measures of *length*. Instead, length is typically provided in **wheel units**,
//! which is defined as "whatever the user measured their wheels with". This gives users a choice
//! of what units to use and keeps everything generally stable while the Rust ecosystem tries to
//! figure out what typed units library to go with.

macro_rules! log {
    ($($arg:tt)*) => {
        #[cfg(debug_assertions)]
        {
            ::std::println!("[ozton-tracking] {}", ::core::format_args!($($arg)*));
        }
    };
}

pub mod gps;
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
/// Implementors of this trait MUST provide their position estimate in the [*cartesian coordinate
/// system*](https://en.wikipedia.org/wiki/Cartesian_coordinate_system). Units are expected to be
/// in *wheel units* — the same units that were used to measure the robot's wheel diameter.
pub trait TracksPosition: Tracking {
    /// Return's the robot's position on a 2D cartesian coordinate plane measured
    /// in wheel units.
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
/// Implementors of this trait MUST provide their angles in a system compatible with cartesian
/// coordinates in *standard position*. This means that anticlockwise motion is considered a
/// positive rotation, and a rotation of zero degrees is inline with the x-axis.
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

/// Creates a shared motor array.
///
/// This macro simplifies the creation of an `Rc<RefCell<[Motor; N]>>` array, which is a shareable
/// wrapper around vexide's non-copyable [`Motor`](vexide::devices::smart::motor::Motor) struct.
///
/// # Examples
///
/// ```ignore
/// let motors = shared_motors![motor1, motor2, motor3];
/// ```
#[macro_export]
macro_rules! shared_motors {
    ( $( $item:expr ),* $(,)?) => {
        {
            use ::std::{cell::RefCell, rc::Rc};

            Rc::new(RefCell::new([$($item,)*]))
        }
    };
}
