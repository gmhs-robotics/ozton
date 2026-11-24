#[cfg(feature = "vex")]
use core::f64::consts::PI;

#[cfg(feature = "vex")]
use ozton_math::uom::si::f64::Length;

pub use super::projection::{DriveWheels, MountedWheel, WheelOdometry};
#[cfg(feature = "vex")]
use crate::RotarySensor;

#[cfg(feature = "vex")]
/// A wheel attached to a rotary sensor for position tracking.
#[derive(Debug, Clone, PartialEq)]
pub struct TrackingWheel<T: RotarySensor> {
    /// Rotary sensor for measuring the wheel's travel.
    pub sensor: T,

    /// Diameter of the wheel.
    pub wheel_diameter: Length,

    /// Signed offset from the drivetrain's center of rotation.
    ///
    /// For a forward wheel, this is X right from The Origin. For a sideways wheel,
    /// this is Y forward from The Origin. Forward travel is +Y; sideways travel is +X.
    /// Heading is CCW from +Y. Both tracking implementations use these conventions.
    pub offset: Length,

    /// External gearing of the wheel.
    ///
    /// Used as a multiplier when determining wheel travel.
    pub gearing: Option<f64>,
}

#[cfg(feature = "vex")]
impl<T: RotarySensor> TrackingWheel<T> {
    /// Creates a new tracking wheel with the given parameters.
    ///
    /// # Parameters
    ///
    /// * `sensor` - The rotary sensor to read wheel rotation from.
    /// * `wheel_diameter` - The diameter of the wheel in linear units.
    /// * `offset` - Distance from wheel to robot's center of rotation.
    /// * `gearing` - Optional gear ratio between sensor and wheel (use None for 1:1 if ungeared).
    pub const fn new(
        sensor: T,
        wheel_diameter: Length,
        offset: Length,
        gearing: Option<f64>,
    ) -> Self {
        Self {
            sensor,
            wheel_diameter,
            offset,
            gearing,
        }
    }

    /// Calculates the total linear distance traveled by this wheel.
    ///
    /// This method uses the wheel's diameter and gear ratio to convert
    /// sensor rotations into linear distance traveled.
    ///
    /// # Errors
    ///
    /// Returns an error if [`RotarySensor::position`] fails.
    pub fn travel(&self) -> Result<Length, T::Error> {
        let wheel_circumference = self.wheel_diameter * PI;

        Ok(self.sensor.position()?.as_turns() * self.gearing.unwrap_or(1.0) * wheel_circumference)
    }
}

impl crate::TrackingSource for MountedWheel {
    fn initialize(&mut self) -> crate::SourceInitialization<'_> {
        if self.is_valid() {
            None
        } else {
            Some(Box::pin(async {
                Err(crate::sampling::TrackingBuildError::WheelGeometry)
            }))
        }
    }

    fn sample(&mut self, sink: &mut crate::MeasurementSink<'_>) {
        if let Some(travel) = MountedWheel::sample(self) {
            sink.push(crate::Measurement::Displacement(
                crate::DisplacementObservation::ProjectedWheel(travel),
            ));
        }
    }

    fn wheel_projection(&self) -> Option<glam::DVec3> {
        Some(self.projection())
    }
}
