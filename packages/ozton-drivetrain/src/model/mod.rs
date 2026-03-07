//! Drivetrain models.
//!
//! This module provides types for modeling a robot's motion capabilities through various drivetrain
//! configurations.

use glam::DVec2 as Vec2;
use ozton_math::desaturate;

mod differential;
mod mecanum;

pub use differential::Differential;
pub use mecanum::Mecanum;

/// A collection of motors driving a wheeled mobile robot.
///
/// Implementors of this trait represent a physical or simulated robot drivetrain and may define
/// additional motion-related functionality.
///
/// This is the base trait for all drivetrain models, such as [`Tank`], [`Arcade`], or
/// [`Holonomic`].
pub trait DrivetrainModel {
    /// Error type returned when the robot fails to move.
    type Error;
}

/// A drivetrain model that supports holonomic inverse kinematics.
///
/// Holonomic drivetrains (such as mecanum or swerve) can move freely in any direction without
/// changing their orientation.
///
/// This interface allows for driving the robot based on a desired translation vector and a
/// rotational component.
pub trait Holonomic: DrivetrainModel {
    /// Drives the robot using a translation vector and rotational power.
    fn drive_vector(&mut self, vector: Vec2, turn: f64) -> Result<(), Self::Error>;
}

/// A drivetrain model that supports "arcade drive" (forward/turn) inverse kinematics.
pub trait Arcade: DrivetrainModel {
    /// Drives the robot using arcade-style controls.
    fn drive_arcade(&mut self, throttle: f64, steer: f64) -> Result<(), Self::Error>;
}

/// A drivetrain model that supports "tank drive" (left/right) inverse kinematics.
pub trait Tank: DrivetrainModel {
    /// Drives the robot using left and right wheel powers.
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), Self::Error>;
}

impl<T: Tank> Arcade for T {
    fn drive_arcade(&mut self, throttle: f64, steer: f64) -> Result<(), Self::Error> {
        let [left, right] = desaturate([throttle + steer, throttle - steer], 1.0);

        self.drive_tank(left, right)
    }
}
