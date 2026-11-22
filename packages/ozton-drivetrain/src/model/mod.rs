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

pub use crate::wheels::{
    DriveConfigError, WheelFeedback, WheelGearing, WheelLoad, WheelSpeedError,
};

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

/// Calibrated cumulative drive-wheel travel in left, right order, positive forward.
pub trait DriveEncoders {
    /// Return the drivetrain's surveyed track width when it is configured.
    fn track_width(&self) -> Option<ozton_math::uom::si::f64::Length> {
        None
    }

    /// Sample both sides once. An unavailable side returns `None`.
    fn tracking_positions(&mut self) -> [Option<ozton_math::uom::si::f64::Length>; 2];
}

/// Commands consumed by a drivetrain during an Ozton competition.
pub trait ProgramDrive {
    type Error;
    type DriverCommand;
    type AutonCommand;

    fn apply_driver(
        &mut self,
        command: Self::DriverCommand,
        dt: std::time::Duration,
    ) -> Result<(), Self::Error>;
    fn apply_auton(
        &mut self,
        command: Self::AutonCommand,
        dt: std::time::Duration,
    ) -> Result<(), Self::Error>;
    fn stop(&mut self) -> Result<(), Self::Error>;

    /// Consume the marker set by a direct driver write through the drivetrain.
    fn take_driver_output(&mut self) -> bool {
        false
    }
}

/// Normalized forward and turn inputs for an arcade drivetrain.
#[derive(Clone, Copy, Debug)]
pub struct ArcadeInput {
    pub throttle: f64,
    pub turn: f64,
    pub(crate) heading: Option<ozton_math::uom::si::f64::Angle>,
    pub(crate) tilt: Option<ozton_math::uom::si::f64::Angle>,
    pub(crate) output_scale: f64,
}

impl ArcadeInput {
    pub const fn new(throttle: f64, turn: f64) -> Self {
        Self {
            throttle,
            turn,
            heading: None,
            tilt: None,
            output_scale: 1.0,
        }
    }

    pub const fn with_orientation(
        mut self,
        heading: Option<ozton_math::uom::si::f64::Angle>,
        tilt: Option<ozton_math::uom::si::f64::Angle>,
    ) -> Self {
        self.heading = heading;
        self.tilt = tilt;
        self
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

impl<T: Tank> Arcade for T {
    fn drive_arcade(&mut self, throttle: f64, steer: f64) -> Result<(), Self::Error> {
        let [left, right] = desaturate([throttle + steer, throttle - steer], 1.0);

        self.drive_tank(left, right)
    }
}
