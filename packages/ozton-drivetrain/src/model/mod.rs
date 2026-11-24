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

pub use crate::{DriveConfigError, WheelFeedback, WheelGearing, WheelSpeedError};

/// Normalized robot-relative actuation. Positive Y drives forward, positive X
/// strafes right on holonomic drives, and positive turn rotates right.
pub trait Drive {
    type Error;

    fn drive_vector(&mut self, vector: Vec2, turn: f64) -> Result<(), Self::Error>;

    /// Immediately remove output, bypassing command transforms where applicable.
    fn stop_now(&mut self) -> Result<(), Self::Error> {
        self.drive_vector(Vec2::ZERO, 0.0)
    }
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

    /// Scale both input axes by a controller-selected fraction before mixing.
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

/// A drive capable of lateral motion.
pub trait Holonomic: Drive {}

/// Robot-relative forward and turn control.
pub struct Arcade;

impl Arcade {
    pub fn drive<D: Drive + ?Sized>(
        drive: &mut D,
        forward: f64,
        turn: f64,
    ) -> Result<(), D::Error> {
        drive.drive_vector(Vec2::new(0.0, forward), turn)
    }
}

/// Left and right side control.
pub struct Tank;

impl Tank {
    pub fn drive<D: Drive + ?Sized>(drive: &mut D, left: f64, right: f64) -> Result<(), D::Error> {
        let (forward, turn) = tank_axes([left, right]);
        Arcade::drive(drive, forward, turn)
    }
}

/// A model that can limit its own commands using its installed actuators' capacity.
pub trait Balancing: Drive {
    /// Reduce commands to match the available motors.
    fn balance(&self, command: &mut [f64; 2]) -> Result<(), Self::Error>;
}

/// Mix normalized arcade axes into tank commands.
pub fn arcade_mix(throttle: f64, turn: f64) -> [f64; 2] {
    desaturate([throttle + turn, throttle - turn], 1.0)
}

pub(crate) fn tank_axes([left, right]: [f64; 2]) -> (f64, f64) {
    (left * 0.5 + right * 0.5, left * 0.5 - right * 0.5)
}
