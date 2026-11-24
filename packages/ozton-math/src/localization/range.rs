//! Ray casting and distance-sensor resets against known field surfaces.

use glam::DVec2;
use uom::{
    ConstZero,
    si::f64::{Length, Time},
};

use crate::geometry::{Pose, direction, rotation};

/// A finite reflecting face along an XY segment, with constant vertical extent.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Surface {
    pub start: DVec2,
    pub end: DVec2,
    pub height: Length,
    pub base_height: Length,
}

impl Surface {
    /// A continuous face extending from the floor to the specified height.
    pub const fn new(start: DVec2, end: DVec2, height: Length) -> Self {
        Self {
            start,
            end,
            height,
            base_height: Length::ZERO,
        }
    }

    /// Raise the entire face without changing its height (e.g. an elevated fixture).
    pub const fn with_base_height(mut self, base_height: Length) -> Self {
        self.base_height = base_height;
        self
    }

    pub fn visible_at(self, height: Length) -> bool {
        (self.base_height..self.base_height + self.height).contains(&height)
    }

    pub fn is_valid(self) -> bool {
        self.start.is_finite()
            && self.end.is_finite()
            && self.start.distance(self.end) > 1e-6
            && self.height.value.is_finite()
            && self.height.value > 0.0
            && self.base_height.value.is_finite()
            && self.base_height.value >= 0.0
            && (self.base_height + self.height).value.is_finite()
    }
}

/// A sensor pose in the robot frame, with X right and Y forward.
pub type Mount = Pose;

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Hit {
    pub surface: usize,
    pub distance: f64,
    pub edge_distance: f64,
    pub normal: DVec2,
    pub incidence: f64,
}

pub fn cast(pose: Pose, mount: Pose, surfaces: &[Surface]) -> Option<Hit> {
    cast_inner(pose, mount, surfaces, None)
}

/// Casts against eligible surfaces among the first 128 map entries. Hit indices
/// always refer to the original map, including when nearer surfaces are masked.
pub fn cast_masked(pose: Pose, mount: Pose, surfaces: &[Surface], mask: u128) -> Option<Hit> {
    cast_inner(pose, mount, surfaces, Some(mask))
}

fn cast_inner(pose: Pose, mount: Pose, surfaces: &[Surface], mask: Option<u128>) -> Option<Hit> {
    let origin = pose.transform_point(mount.position);
    let beam = direction(pose.heading + mount.heading);
    let mut nearest = None;
    let mut nearest_distance = f64::INFINITY;
    for (index, surface) in surfaces.iter().enumerate() {
        if mask.is_some_and(|mask| index >= u128::BITS as usize || mask & (1u128 << index) == 0) {
            continue;
        }
        let Some((distance, fraction)) =
            intersection(origin, beam, surface.start, surface.end - surface.start)
        else {
            continue;
        };
        if distance >= nearest_distance {
            continue;
        }
        let segment = surface.end - surface.start;
        let length_squared = segment.length_squared();
        nearest_distance = distance;
        nearest = Some((index, segment, length_squared, fraction));
    }
    let (surface, segment, length_squared, fraction) = nearest?;
    let length = libm::sqrt(length_squared);
    let normal = segment.perp() / length;
    Some(Hit {
        surface,
        distance: nearest_distance,
        edge_distance: fraction.min(1.0 - fraction) * length,
        normal,
        incidence: beam.dot(normal),
    })
}

/// Nearest eligible distance for a world-space ray with a unit direction.
/// Iterate only visible faces and skip normal/edge calculations used by DSR.
pub fn ray_distance(
    origin: DVec2,
    beam: DVec2,
    surfaces: &[Surface],
    mut mask: u128,
) -> Option<f64> {
    let mut nearest = f64::INFINITY;
    while mask != 0 {
        let index = mask.trailing_zeros() as usize;
        let Some(surface) = surfaces.get(index) else {
            break;
        };
        mask &= mask - 1;
        if let Some((distance, _)) =
            intersection(origin, beam, surface.start, surface.end - surface.start)
        {
            nearest = nearest.min(distance);
        }
    }
    nearest.is_finite().then_some(nearest)
}

fn intersection(origin: DVec2, beam: DVec2, start: DVec2, segment: DVec2) -> Option<(f64, f64)> {
    let length_squared = segment.length_squared();
    let denominator = beam.perp_dot(segment);
    if length_squared <= 1e-12 || denominator * denominator <= length_squared * 1e-18 {
        return None;
    }
    let offset = start - origin;
    let distance = offset.perp_dot(segment) / denominator;
    let fraction = offset.perp_dot(beam) / denominator;
    (distance > 0.0 && distance.is_finite() && (0.0..=1.0).contains(&fraction))
        .then_some((distance, fraction))
}

/// Maximum observation age in seconds and geometry/quality gates for a distance sensor.
#[derive(Debug, Clone, Copy)]
pub struct RangeGate {
    pub min_range: Length,
    pub max_range: Length,
    pub min_confidence: f64,
    pub max_age: Time,
    pub min_incidence: f64,
    pub edge_margin: Length,
}

impl Default for RangeGate {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl RangeGate {
    pub const DEFAULT: Self = Self {
        min_range: crate::qty!(0.05 m),
        max_range: crate::qty!(2.0 m),
        min_confidence: 0.5,
        max_age: Time {
            value: 0.08,
            ..Time::ZERO
        },
        min_incidence: 0.6,
        edge_margin: crate::qty!(0.1 m),
    };

    pub fn is_valid(self) -> bool {
        [
            self.min_range.value,
            self.max_range.value,
            self.min_confidence,
            self.max_age.value,
            self.min_incidence,
            self.edge_margin.value,
        ]
        .iter()
        .all(|value| value.is_finite() && *value >= 0.0)
            && self.min_range < self.max_range
            && self.max_age.value > 0.0
            && self.min_confidence <= 1.0
            && (0.001..=1.0).contains(&self.min_incidence)
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
    Uncertain,
}

#[derive(Debug, Clone, Copy)]
pub struct RangeReading {
    pub distance: Length,
    pub confidence: f64,
    pub age: Time,
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
        if !(0.0..=gate.max_age.value).contains(&self.age.value) {
            return Err(Rejection::Stale);
        }
        Ok(())
    }
}

/// A scalar position constraint n·Δp = displacement, without an invented tangential measurement.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Constraint {
    pub hit: Hit,
    pub residual: f64,
    pub displacement: f64,
}

pub fn constrain(
    pose: Pose,
    mount: Pose,
    reading: RangeReading,
    gate: RangeGate,
    surfaces: &[Surface],
    expected_surface: Option<usize>,
) -> Result<Constraint, Rejection> {
    reading.validate(gate)?;
    let hit = cast(pose, mount, surfaces).ok_or(Rejection::NoSurface)?;
    constrain_hit(hit, reading, gate, expected_surface)
}

/// Validates a range against the nearest eligible surface with one ray cast.
pub fn constrain_masked(
    pose: Pose,
    mount: Pose,
    reading: RangeReading,
    gate: RangeGate,
    surfaces: &[Surface],
    expected_surface: Option<usize>,
    mask: u128,
) -> Result<Constraint, Rejection> {
    reading.validate(gate)?;
    let hit = cast_masked(pose, mount, surfaces, mask).ok_or(Rejection::NoSurface)?;
    constrain_hit(hit, reading, gate, expected_surface)
}

fn constrain_hit(
    hit: Hit,
    reading: RangeReading,
    gate: RangeGate,
    expected_surface: Option<usize>,
) -> Result<Constraint, Rejection> {
    if expected_surface.is_some_and(|expected| expected != hit.surface) {
        return Err(Rejection::WrongSurface);
    }
    if hit.edge_distance < gate.edge_margin.value {
        return Err(Rejection::NearEdge);
    }
    if hit.incidence.abs() < gate.min_incidence {
        return Err(Rejection::Grazing);
    }
    let residual = reading.distance.value - hit.distance;
    let lateral_uncertainty =
        residual.abs() * libm::sqrt((1.0 - hit.incidence * hit.incidence).max(0.0));
    if hit.edge_distance < gate.edge_margin.value + lateral_uncertainty {
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

/// Weighted translation fit over every eligible beam. Rejects contradictory beams,
/// rather than selecting whichever sensor happens to appear first. Noise is in metres
/// along the wall normal. Rank one preserves the unobserved tangent exactly.
pub fn fit_translation(constraints: &[(Constraint, f64)]) -> Option<(DVec2, u8)> {
    let mut xx = 0.0;
    let mut xy = 0.0;
    let mut yy = 0.0;
    let mut rhs = DVec2::ZERO;
    for &(constraint, sigma) in constraints {
        if !sigma.is_finite() || sigma <= 0.0 || !constraint.displacement.is_finite() {
            return None;
        }
        let n = constraint.hit.normal;
        if !n.is_finite() || (n.length_squared() - 1.0).abs() > 1e-6 {
            return None;
        }
        let w = 1.0 / (sigma * sigma);
        xx += w * n.x * n.x;
        xy += w * n.x * n.y;
        yy += w * n.y * n.y;
        rhs += w * n * constraint.displacement;
    }
    let trace = xx + yy;
    if !trace.is_finite() || trace <= 0.0 {
        return None;
    }
    let determinant = xx * yy - xy * xy;
    let (change, axes) = if determinant > 0.05 * trace * trace {
        (
            DVec2::new(yy * rhs.x - xy * rhs.y, xx * rhs.y - xy * rhs.x) / determinant,
            2,
        )
    } else {
        // Principal direction of the symmetric normal matrix; sign does not matter.
        let angle = 0.5 * libm::atan2(2.0 * xy, xx - yy);
        let n = rotation(crate::qty!((angle) rad));
        let eigenvalue = xx * n.x * n.x + 2.0 * xy * n.x * n.y + yy * n.y * n.y;
        (n * (rhs.dot(n) / eigenvalue), 1)
    };
    if !change.is_finite()
        || constraints.iter().any(|(c, sigma)| {
            (c.hit.normal.dot(change) - c.displacement).abs() > (3.0 * sigma).max(0.025)
        })
    {
        return None;
    }
    Some((change, axes))
}

#[cfg(test)]
mod tests {
    use uom::si::time::second as time_second;

    use super::*;

    #[test]
    fn surface_height_and_elevated_face_visibility() {
        let wall = Surface::new(DVec2::ZERO, DVec2::X, crate::qty!(5.125 in));
        assert!(wall.is_valid());
        assert!(wall.visible_at(crate::qty!(3.61811 in)));
        assert!(!wall.visible_at(crate::qty!(8.125 in)));
        assert!(!wall.visible_at(crate::qty!(5.125 in)));
        let raised = wall.with_base_height(crate::qty!(2 in));
        assert!(!raised.visible_at(crate::qty!(1 in)));
        assert!(raised.visible_at(crate::qty!(6 in)));
        assert!(!Surface::new(DVec2::ZERO, DVec2::X, crate::qty!(0 in)).is_valid());
    }

    #[test]
    fn oblique_wall_only_constrains_normal() {
        let surface = Surface::new(
            DVec2::new(1.0, -1.0),
            DVec2::new(1.0, 1.0),
            crate::qty!(1 m),
        );
        let pose = Pose::new(
            crate::geometry::point(crate::qty!(0.1 m), crate::qty!(0.1 m)),
            crate::qty!(0.3 rad),
        );
        let mount = Pose {
            position: DVec2::new(0.1, 0.0),
            heading: crate::qty!((-core::f64::consts::FRAC_PI_2) rad),
        };
        let true_pose = Pose::new(
            crate::geometry::point(crate::qty!(0.0 m), crate::qty!(0.1 m)),
            crate::qty!(0.3 rad),
        );
        let distance = cast(true_pose, mount, &[surface]).unwrap().distance;
        let reading = RangeReading {
            distance: crate::qty!((distance) m),
            confidence: 1.0,
            age: Time::new::<time_second>(0.0),
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
