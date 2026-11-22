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

pub use crate::{DriveConfigError, WheelFeedback, WheelGearing, WheelLoad, WheelSpeedError};

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

pub use ozton_tracking::encoders::DriveEncoders;

/// Normalized forward and turn inputs for an arcade drivetrain.
#[derive(Clone, Copy, Debug)]
pub struct ArcadeInput {
    pub throttle: f64,
    pub turn: f64,
    pub(crate) output_scale: f64,
}

impl ArcadeInput {
    pub const fn new(throttle: f64, turn: f64) -> Self {
        Self {
            throttle,
            turn,
            output_scale: 1.0,
        }
    }

    /// Multiply the mixed wheel outputs by a controller-selected fraction.
    pub const fn with_output_scale(mut self, scale: f64) -> Self {
        self.output_scale = if scale.is_finite() {
            scale.clamp(0.0, 1.0)
        } else {
            0.0
        };
        self
    }

    pub const fn scaled_axes(self) -> (f64, f64) {
        (
            self.throttle * self.output_scale,
            self.turn * self.output_scale,
        )
    }
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

    /// Immediately remove drivetrain output on disable or fault.
    fn stop_now(&mut self) -> Result<(), Self::Error> {
        self.drive_tank(0.0, 0.0)
    }
}

/// A model that can limit its own commands using its installed actuators' capacity.
pub trait Balancing: DrivetrainModel {
    type Command;

    /// Apply a balanced command, attempting all motor writes even if a port fails.
    fn drive_balanced(&mut self, command: Self::Command) -> Result<(), Self::Error>;
}

/// Mix normalized arcade axes into tank commands.
pub fn arcade_mix(throttle: f64, turn: f64) -> [f64; 2] {
    desaturate([throttle + turn, throttle - turn], 1.0)
}

impl<T: Tank> Arcade for T {
    fn drive_arcade(&mut self, throttle: f64, steer: f64) -> Result<(), Self::Error> {
        let [left, right] = arcade_mix(throttle, steer);

        self.drive_tank(left, right)
    }
}
