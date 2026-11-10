//! Ray casting and distance-sensor resets against known field surfaces.

use glam::DVec2;

use crate::geometry::{Pose, direction};

/// A finite, stationary reflecting surface in the field plane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Surface {
    pub start: DVec2,
    pub end: DVec2,
}

impl Surface {
    pub const fn new(start_x: f64, start_y: f64, end_x: f64, end_y: f64) -> Self {
        Self {
            start: DVec2::new(start_x, start_y),
            end: DVec2::new(end_x, end_y),
        }
    }

    pub fn is_valid(self) -> bool {
        self.start.is_finite() && self.end.is_finite() && self.start.distance(self.end) > 1e-6
    }
}

/// Sensor pose relative to the robot's localization origin.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mount {
    pub offset: DVec2,
    pub heading: f64,
}

impl Mount {
    pub const fn new(x: f64, y: f64, heading: f64) -> Self {
        Self {
            offset: DVec2::new(x, y),
            heading,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hit {
    pub surface: usize,
    pub distance: f64,
    pub edge_distance: f64,
    pub normal: DVec2,
    pub incidence: f64,
}

pub fn cast(pose: Pose, mount: Mount, surfaces: &[Surface]) -> Option<Hit> {
    let origin = pose.transform_point(mount.offset);
    let beam = direction(pose.heading + mount.heading);
    surfaces
        .iter()
        .enumerate()
        .filter_map(|(index, surface)| {
            let segment = surface.end - surface.start;
            let length = segment.length();
            let denominator = beam.perp_dot(segment);
            if length <= 1e-6 || denominator.abs() <= length * 1e-9 {
                return None;
            }
            let offset = surface.start - origin;
            let distance = offset.perp_dot(segment) / denominator;
            let fraction = offset.perp_dot(beam) / denominator;
            if distance <= 0.0 || !distance.is_finite() || !(0.0..=1.0).contains(&fraction) {
                return None;
            }
            let normal = segment.perp() / length;
            Some(Hit {
                surface: index,
                distance,
                edge_distance: fraction.min(1.0 - fraction) * length,
                normal,
                incidence: beam.dot(normal),
            })
        })
        .min_by(|left, right| left.distance.total_cmp(&right.distance))
}

/// Maximum observation age in seconds and geometry/quality gates for a distance sensor.
#[derive(Debug, Clone, Copy)]
pub struct RangeGate {
    pub min_range: f64,
    pub max_range: f64,
    pub min_confidence: f64,
    pub max_age: f64,
    pub min_incidence: f64,
    pub edge_margin: f64,
    pub max_residual: f64,
}

impl Default for RangeGate {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl RangeGate {
    pub const DEFAULT: Self = Self {
        min_range: 0.05,
        max_range: 2.0,
        min_confidence: 0.5,
        max_age: 0.08,
        min_incidence: 0.6,
        edge_margin: 0.1,
        max_residual: 0.18,
    };

    pub fn is_valid(self) -> bool {
        [
            self.min_range,
            self.max_range,
            self.min_confidence,
            self.max_age,
            self.min_incidence,
            self.edge_margin,
            self.max_residual,
        ]
        .iter()
        .all(|value| value.is_finite() && *value >= 0.0)
            && self.min_range < self.max_range
            && self.max_age > 0.0
            && self.min_confidence <= 1.0
            && (0.001..=1.0).contains(&self.min_incidence)
            && self.max_residual > 0.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rejection {
    Missing,
    Duplicate,
    Occluded,
    OutOfRange,
    LowConfidence,
    Stale,
    NoSurface,
    WrongSurface,
    NearEdge,
    Grazing,
    Residual,
    Uncertain,
}

#[derive(Debug, Clone, Copy)]
pub struct RangeReading {
    pub distance: f64,
    pub confidence: f64,
    pub age: f64,
    pub occluded: bool,
}

impl RangeReading {
    pub fn validate(self, gate: RangeGate) -> Result<(), Rejection> {
        if self.occluded {
            return Err(Rejection::Occluded);
        }
        if !gate.is_valid() || !(gate.min_range..=gate.max_range).contains(&self.distance) {
            return Err(Rejection::OutOfRange);
        }
        if !(gate.min_confidence..=1.0).contains(&self.confidence) {
            return Err(Rejection::LowConfidence);
        }
        if !(0.0..=gate.max_age).contains(&self.age) {
            return Err(Rejection::Stale);
        }
        Ok(())
    }
}

/// A scalar position constraint n·Δp = displacement, without an invented tangential measurement.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Constraint {
    pub hit: Hit,
    pub residual: f64,
    pub displacement: f64,
}

pub fn constrain(
    pose: Pose,
    mount: Mount,
    reading: RangeReading,
    gate: RangeGate,
    surfaces: &[Surface],
    expected_surface: Option<usize>,
) -> Result<Constraint, Rejection> {
    reading.validate(gate)?;
    let hit = cast(pose, mount, surfaces).ok_or(Rejection::NoSurface)?;
    if expected_surface.is_some_and(|expected| expected != hit.surface) {
        return Err(Rejection::WrongSurface);
    }
    if hit.edge_distance < gate.edge_margin {
        return Err(Rejection::NearEdge);
    }
    if hit.incidence.abs() < gate.min_incidence {
        return Err(Rejection::Grazing);
    }
    let residual = reading.distance - hit.distance;
    if residual.abs() > gate.max_residual {
        return Err(Rejection::Residual);
    }
    let lateral_uncertainty =
        residual.abs() * libm::sqrt((1.0 - hit.incidence * hit.incidence).max(0.0));
    if hit.edge_distance < gate.edge_margin + lateral_uncertainty {
        return Err(Rejection::NearEdge);
    }
    Ok(Constraint {
        hit,
        residual,
        displacement: -residual * hit.incidence,
    })
}

/// Uses a second constraint only when it independently observes the other planar axis.
pub fn reset_translation(first: Constraint, second: Option<Constraint>) -> DVec2 {
    let normal = first.hit.normal;
    if let Some(second) = second {
        let other = second.hit.normal;
        let determinant = normal.perp_dot(other);
        if determinant.abs() >= 0.5 {
            return (normal.perp() * second.displacement - other.perp() * first.displacement)
                / determinant;
        }
    }
    normal * first.displacement
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn oblique_wall_only_constrains_normal() {
        let surface = Surface {
            start: DVec2::new(1.0, -1.0),
            end: DVec2::new(1.0, 1.0),
        };
        let pose = Pose::new(0.1, 0.1, 0.3);
        let mount = Mount {
            offset: DVec2::new(0.1, 0.0),
            heading: 0.0,
        };
        let true_pose = Pose::new(0.0, 0.1, 0.3);
        let distance = cast(true_pose, mount, &[surface]).unwrap().distance;
        let reading = RangeReading {
            distance,
            confidence: 1.0,
            age: 0.0,
            occluded: false,
        };
        let constraint = constrain(
            pose,
            mount,
            reading,
            RangeGate::default(),
            &[surface],
            Some(0),
        )
        .unwrap();
        assert!(reset_translation(constraint, None).distance(DVec2::new(-0.1, 0.0)) < 1e-10);
        let occluded = RangeReading {
            occluded: true,
            ..reading
        };
        assert_eq!(
            constrain(
                pose,
                mount,
                occluded,
                RangeGate::default(),
                &[surface],
                None
            ),
            Err(Rejection::Occluded)
        );
    }
}
