//! Odometry from mounted tracking wheels and optional differential drive encoders.

use glam::DVec3;
use ozton_math::{
    geometry::direction,
    uom::si::f64::{Angle, Length},
};

use crate::{ReadPosition, mount::Vec3, odometric::wheel::TrackingWheel, sampling::WheelSource};

/// A tracking wheel at a surveyed position. Positive travel follows its bearing,
/// measured counterclockwise from robot forward.
pub struct MountedWheel<W> {
    source: W,
    projection: DVec3,
}

impl<S: ReadPosition> MountedWheel<TrackingWheel<S>> {
    /// Mount the rotary tracking wheel at a surveyed XYZ position and heading.
    pub fn at_mount(
        sensor: S,
        diameter: Length,
        gearing: Option<f64>,
        (position, bearing): (Vec3<Length>, Angle),
    ) -> Self {
        Self::from_rotary(sensor, diameter, gearing, bearing, position)
    }

    /// Mount a rotary encoder on a tracking wheel. Gearing is wheel turns
    /// per encoder turn. Pass `None` for a direct drive.
    pub fn from_rotary(
        sensor: S,
        diameter: Length,
        gearing: Option<f64>,
        bearing: Angle,
        position: Vec3<Length>,
    ) -> Self {
        Self::new(
            TrackingWheel::new(sensor, diameter, Length::default(), gearing),
            position,
            bearing,
        )
    }
}

impl<W: WheelSource> MountedWheel<W> {
    /// Use the right and forward coordinates relative to the tracked origin.
    /// The vertical coordinate does not affect planar odometry.
    pub fn new(source: W, position: Vec3<Length>, bearing: Angle) -> Self {
        let axis = direction(bearing);
        let rotation = axis.y * position.right.value - axis.x * position.forward.value;
        Self {
            source,
            projection: axis.extend(rotation),
        }
    }
    /// Geometry of the sensor observation in the chassis frame.
    pub fn projection(&self) -> DVec3 {
        self.projection
    }

    pub fn is_valid(&self) -> bool {
        self.projection.is_finite() && self.source.valid()
    }

    /// Read cumulative travel. The localization stage computes motion between samples.
    pub fn sample(&self) -> Option<Length> {
        self.source.read_travel()
    }
}
