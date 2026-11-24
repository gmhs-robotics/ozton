//! Field-referenced measurements, such as a fix reported by a GPS receiver.

use std::time::Duration;

use glam::DVec2;
use ozton_math::uom::si::f64::Angle;

/// Observation of the chassis origin in field metres and counterclockwise heading.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PoseObservation {
    pub time: Duration,
    pub position: DVec2,
    pub heading: Option<Angle>,
}
