use glam::DVec3;
use ozton_math::uom::si::f64::{AngularVelocity, Length};

use super::{DriveConfigError, SharedMotors, WheelFeedback, WheelGearing, Wheels, positive};
use crate::{DisplacementObservation, Measurement, MeasurementSink, TrackingSource};

/// Calibration and observation state for drive-wheel encoders. The drive model
/// shares motor handles with this source.
pub struct DifferentialTracking {
    track_width: Length,
    wheels: Wheels,
    motors: [SharedMotors; 2],
    feedback: [Option<WheelFeedback>; 2],
}

impl DifferentialTracking {
    /// Calibrate with a gear ratio per motor, in left and right group order.
    pub fn new(
        motors: &[SharedMotors; 2],
        track_width: Length,
        diameter: Length,
        gearing: &[&[WheelGearing]],
    ) -> Result<Self, DriveConfigError> {
        Self::with_wheels(
            motors.clone(),
            track_width,
            Wheels::geared(diameter, motors, gearing)?,
        )
    }

    /// Infer ratios from the common free wheel speed and the motor cartridges.
    pub fn inferred(
        motors: &[SharedMotors; 2],
        track_width: Length,
        diameter: Length,
        free_speed: AngularVelocity,
    ) -> Result<Self, DriveConfigError> {
        Self::with_wheels(
            motors.clone(),
            track_width,
            Wheels::inferred(diameter, motors, free_speed)?,
        )
    }

    fn with_wheels(
        motors: [SharedMotors; 2],
        track_width: Length,
        wheels: Wheels,
    ) -> Result<Self, DriveConfigError> {
        positive(track_width.value, DriveConfigError::TrackWidth)?;
        Ok(Self {
            track_width,
            wheels,
            motors,
            feedback: [None; 2],
        })
    }

    pub fn track_width(&self) -> Length {
        self.track_width
    }

    pub fn wheels(&self) -> &Wheels {
        &self.wheels
    }

    pub fn read_wheels(&mut self) -> [Option<WheelFeedback>; 2] {
        self.feedback = self.wheels.sample(&self.motors);
        self.feedback
    }

    pub fn feedback(&self) -> [Option<WheelFeedback>; 2] {
        self.feedback
    }
}

impl TrackingSource for DifferentialTracking {
    fn sample(&mut self, sink: &mut MeasurementSink<'_>) {
        let feedback = self.read_wheels();
        let positions = feedback.map(|sample| sample.map(|wheel| wheel.position));
        sink.push(Measurement::Displacement(
            DisplacementObservation::Differential(positions),
        ));
        if let [Some(left), Some(right)] = feedback {
            let [left, right] = [left.velocity.value, right.velocity.value];
            sink.push(Measurement::Velocity(DVec3::Y * ((left + right) * 0.5)));
            sink.push(Measurement::Rotation(
                DVec3::Z * ((right - left) / self.track_width.value),
            ));
        }
    }

    fn differential(&self) -> Option<&Self> {
        Some(self)
    }

    fn differential_mut(&mut self) -> Option<&mut Self> {
        Some(self)
    }
}
