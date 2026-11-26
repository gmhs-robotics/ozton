//! Drivetrain models.
//!
//! This module provides types for modeling a robot's motion capabilities through various drivetrain
//! configurations.

use glam::DVec2 as Vec2;
use ozton_math::desaturate;

mod driver;

#[cfg(feature = "vex")]
pub(crate) mod actuation;
#[cfg(feature = "vex")]
mod differential;
#[cfg(feature = "vex")]
mod mecanum;

#[cfg(feature = "vex")]
pub use differential::Differential;
#[cfg(feature = "vex")]
pub use mecanum::Mecanum;

pub use crate::WheelFeedback;
#[cfg(feature = "vex")]
pub use crate::{DriveConfigError, WheelGearing, WheelSpeedError};

/// Normalized robot-relative actuation. Positive Y drives forward, positive X
/// strafes right on holonomic drives, and positive turn rotates right.
pub trait Drive {
    type Error;

    fn drive_vector(&mut self, vector: Vec2, turn: f64) -> Result<(), Self::Error>;

    /// Drive with normalized forward and turn axes.
    fn arcade(&mut self, forward: f64, turn: f64) -> Result<(), Self::Error> {
        driver::drive_command(self, Vec2::new(0.0, forward), turn, "arcade")
    }

    /// Drive with normalized left and right axes.
    fn tank(&mut self, left: f64, right: f64) -> Result<(), Self::Error> {
        let (forward, turn) = tank_axes([left, right]);
        driver::drive_command(self, Vec2::new(0.0, forward), turn, "tank")
    }

    /// Immediately remove output, bypassing command transforms where applicable.
    fn stop_now(&mut self) -> Result<(), Self::Error> {
        self.drive_vector(Vec2::ZERO, 0.0)
    }
}

pub use ozton_tracking::odometric::DriveEncoders;

/// Explicit geometry policy required by field-relative motion.
/// A drivetrain with no configured geometry does not implement this capability.
///
/// ```compile_fail
/// use ozton_drivetrain::model::MotionGeometry;
/// fn require<G: MotionGeometry>() {}
/// require::<()>();
/// ```
pub trait MotionGeometry {
    /// None explicitly means no footprint, never an omitted configuration.
    fn footprint(&self) -> Option<crate::Footprint>;
}

impl MotionGeometry for crate::Footprint {
    fn footprint(&self) -> Option<crate::Footprint> {
        Some(*self)
    }
}

/// Explicit permission to use pose motion without a configured footprint.
#[derive(Clone, Copy, Debug, Default)]
pub struct UnboundedMotion;

impl MotionGeometry for UnboundedMotion {
    fn footprint(&self) -> Option<crate::Footprint> {
        None
    }
}

/// A model capable of lateral motion.
pub trait LateralDrive: Drive {
    /// Drive with normalized right and forward translation and a normalized turn.
    fn holonomic(&mut self, translation: Vec2, turn: f64) -> Result<(), Self::Error> {
        driver::drive_command(self, translation, turn, "holonomic")
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

/// Convert two differential wheel commands to forward and turn axes.
pub fn tank_axes([left, right]: [f64; 2]) -> (f64, f64) {
    (left * 0.5 + right * 0.5, left * 0.5 - right * 0.5)
}
