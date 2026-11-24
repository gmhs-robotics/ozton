use glam::DVec2;

use crate::uom::si::f64::Angle;

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
pub trait TracksPosition {
    /// Returns The Origin's field position: X right, Y forward, in metres.
    fn position(&self) -> DVec2;
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
pub trait TracksHeading {
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
pub trait TracksVelocity {
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
pub trait TracksForwardTravel {
    /// Returns the signed forward wheel travel of the robot in wheel units.
    ///
    /// Positive values indicate forward movement from the origin of tracking, while negative
    /// values indicate backwards movement from the origin.
    fn forward_travel(&self) -> f64;
}
