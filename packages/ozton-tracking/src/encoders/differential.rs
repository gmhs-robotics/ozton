use ozton_math::uom::si::f64::{AngularVelocity, Length};
use vexide::prelude::Motor;

use super::{DriveConfigError, WheelFeedback, WheelGearing, Wheels, positive};

/// Calibration and observation state for drive-wheel encoders. The drive model
/// owns the motors and lends them to this source when a sample is needed.
pub struct DifferentialTracking {
    track_width: Length,
    wheels: Wheels,
    feedback: [Option<WheelFeedback>; 2],
}

impl DifferentialTracking {
    /// Calibrate with a gear ratio per motor, in left and right group order.
    pub fn new(
        motors: &[Box<[Motor]>],
        track_width: Length,
        diameter: Length,
        gearing: &[&[WheelGearing]],
    ) -> Result<Self, DriveConfigError> {
        Self::with_wheels(track_width, Wheels::geared(diameter, motors, gearing)?)
    }

    /// Infer ratios from the common free wheel speed and the motor cartridges.
    pub fn inferred(
        motors: &[Box<[Motor]>],
        track_width: Length,
        diameter: Length,
        free_speed: AngularVelocity,
    ) -> Result<Self, DriveConfigError> {
        Self::with_wheels(track_width, Wheels::inferred(diameter, motors, free_speed)?)
    }

    fn with_wheels(track_width: Length, wheels: Wheels) -> Result<Self, DriveConfigError> {
        positive(track_width.value, DriveConfigError::TrackWidth)?;
        Ok(Self {
            track_width,
            wheels,
            feedback: [None; 2],
        })
    }

    pub fn track_width(&self) -> Length {
        self.track_width
    }

    pub fn wheels(&self) -> &Wheels {
        &self.wheels
    }

    pub fn sample(&mut self, groups: &[Box<[Motor]>]) -> [Option<WheelFeedback>; 2] {
        self.feedback = self.wheels.sample(groups);
        self.feedback
    }

    pub fn feedback(&self) -> [Option<WheelFeedback>; 2] {
        self.feedback
    }
}
