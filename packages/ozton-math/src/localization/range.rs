//! Ray casting and distance-sensor resets against known field surfaces.

use glam::DVec2;
use uom::{
    ConstZero,
    si::{
        angle::radian,
        f64::{Angle, Length, Time},
    },
};

use crate::geometry::{Pose, direction, rotation};

/// A finite, stationary reflecting surface in the field plane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Surface {
    pub start: DVec2,
    pub end: DVec2,
}

impl Surface {
    pub const fn new(start_x: Length, start_y: Length, end_x: Length, end_y: Length) -> Self {
        Self {
            start: DVec2::new(start_x.value, start_y.value),
            end: DVec2::new(end_x.value, end_y.value),
        }
    }

    pub fn is_valid(self) -> bool {
        self.start.is_finite() && self.end.is_finite() && self.start.distance(self.end) > 1e-6
    }
}

/// Horizontal sensor mount relative to The Origin: X right, Y forward; CCW bearing from +Y.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mount {
    pub offset: DVec2,
    pub heading: f64,
}

impl Mount {
    pub const fn new(x: Length, y: Length, heading: Angle) -> Self {
        Self {
            offset: DVec2::new(x.value, y.value),
            heading: heading.value,
        }
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Hit {
    pub surface: usize,
    pub distance: f64,
    pub edge_distance: f64,
    pub normal: DVec2,
    pub incidence: f64,
}

pub fn cast(pose: Pose, mount: Mount, surfaces: &[Surface]) -> Option<Hit> {
    cast_inner(pose, mount, surfaces, None)
}

/// Casts against eligible surfaces among the first 32 map entries. Hit indices
/// always refer to the original map, including when nearer surfaces are masked.
pub fn cast_masked(pose: Pose, mount: Mount, surfaces: &[Surface], mask: u32) -> Option<Hit> {
    cast_inner(pose, mount, surfaces, Some(mask))
}

fn cast_inner(pose: Pose, mount: Mount, surfaces: &[Surface], mask: Option<u32>) -> Option<Hit> {
    let origin = pose.transform_point(mount.offset);
    let beam = direction(Angle::new::<radian>(pose.heading + mount.heading));
    let mut nearest = None;
    let mut nearest_distance = f64::INFINITY;
    for (index, surface) in surfaces.iter().enumerate() {
        if mask.is_some_and(|mask| index >= 32 || mask & (1u32 << index) == 0) {
            continue;
        }
        let segment = surface.end - surface.start;
        let length_squared = segment.length_squared();
        let denominator = beam.perp_dot(segment);
        if length_squared <= 1e-12 || denominator * denominator <= length_squared * 1e-18 {
            continue;
        }
        let offset = surface.start - origin;
        let distance = offset.perp_dot(segment) / denominator;
        let fraction = offset.perp_dot(beam) / denominator;
        if distance <= 0.0
            || !distance.is_finite()
            || distance >= nearest_distance
            || !(0.0..=1.0).contains(&fraction)
        {
            continue;
        }
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

/// Maximum observation age in seconds and geometry/quality gates for a distance sensor.
#[derive(Debug, Clone, Copy)]
pub struct RangeGate {
    pub min_range: Length,
    pub max_range: Length,
    pub min_confidence: f64,
    pub max_age: Time,
    pub min_incidence: f64,
    pub edge_margin: Length,
    pub max_residual: Length,
}

impl Default for RangeGate {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl RangeGate {
    pub const DEFAULT: Self = Self {
        min_range: Length {
            value: 0.05,
            ..Length::ZERO
        },
        max_range: Length {
            value: 2.0,
            ..Length::ZERO
        },
        min_confidence: 0.5,
        max_age: Time {
            value: 0.08,
            ..Time::ZERO
        },
        min_incidence: 0.6,
        edge_margin: Length {
            value: 0.1,
            ..Length::ZERO
        },
        max_residual: Length {
            value: 0.18,
            ..Length::ZERO
        },
    };

    pub fn is_valid(self) -> bool {
        [
            self.min_range.value,
            self.max_range.value,
            self.min_confidence,
            self.max_age.value,
            self.min_incidence,
            self.edge_margin.value,
            self.max_residual.value,
        ]
        .iter()
        .all(|value| value.is_finite() && *value >= 0.0)
            && self.min_range < self.max_range
            && self.max_age.value > 0.0
            && self.min_confidence <= 1.0
            && (0.001..=1.0).contains(&self.min_incidence)
            && self.max_residual.value > 0.0
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
    mount: Mount,
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
    mount: Mount,
    reading: RangeReading,
    gate: RangeGate,
    surfaces: &[Surface],
    expected_surface: Option<usize>,
    mask: u32,
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
    if residual.abs() > gate.max_residual.value {
        return Err(Rejection::Residual);
    }
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
        let n = rotation(Angle::new::<radian>(angle));
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
    use uom::si::{length::meter, time::second as time_second};

    use super::*;

    #[test]
    fn masked_cast_matches_independent_segment_casts_and_preserves_indices() {
        let mut map = [Surface::new(
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.0),
        ); 33];
        map[0] = Surface::new(
            Length::new::<meter>(-2.0),
            Length::new::<meter>(0.8),
            Length::new::<meter>(2.0),
            Length::new::<meter>(0.8),
        );
        map[4] = Surface::new(
            Length::new::<meter>(-2.0),
            Length::new::<meter>(1.0),
            Length::new::<meter>(2.0),
            Length::new::<meter>(1.0),
        );
        map[31] = Surface::new(
            Length::new::<meter>(-2.0),
            Length::new::<meter>(1.2),
            Length::new::<meter>(2.0),
            Length::new::<meter>(1.2),
        );
        map[32] = Surface::new(
            Length::new::<meter>(-2.0),
            Length::new::<meter>(0.5),
            Length::new::<meter>(2.0),
            Length::new::<meter>(0.5),
        );
        for yaw in [-0.3, 0.0, 0.3] {
            for mask in [0, 1, 1 << 4, 1 << 31, (1 << 4) | (1 << 31), u32::MAX] {
                let pose = Pose::new(
                    Length::new::<meter>(0.1),
                    Length::new::<meter>(-0.2),
                    Angle::new::<radian>(yaw),
                );
                let mount = Mount::new(
                    Length::new::<meter>(0.05),
                    Length::new::<meter>(-0.03),
                    Angle::new::<radian>(0.1),
                );
                let expected = map
                    .iter()
                    .enumerate()
                    .take(32)
                    .filter(|(i, _)| mask & (1u32 << i) != 0)
                    .filter_map(|(i, surface)| {
                        cast(pose, mount, &[*surface]).map(|mut hit| {
                            hit.surface = i;
                            hit
                        })
                    })
                    .min_by(|a, b| a.distance.total_cmp(&b.distance));
                assert_eq!(cast_masked(pose, mount, &map, mask), expected);
            }
        }
        assert_eq!(
            cast(
                Pose::default(),
                Mount::new(
                    Length::new::<meter>(0.0),
                    Length::new::<meter>(0.0),
                    Angle::new::<radian>(0.0)
                ),
                &map
            )
            .unwrap()
            .surface,
            32
        );
        assert_eq!(
            cast_masked(
                Pose::default(),
                Mount::new(
                    Length::new::<meter>(0.0),
                    Length::new::<meter>(0.0),
                    Angle::new::<radian>(0.0)
                ),
                &map,
                u32::MAX
            )
            .unwrap()
            .surface,
            0
        );
    }

    #[test]
    fn oblique_wall_only_constrains_normal() {
        let surface = Surface {
            start: DVec2::new(1.0, -1.0),
            end: DVec2::new(1.0, 1.0),
        };
        let pose = Pose::new(
            Length::new::<meter>(0.1),
            Length::new::<meter>(0.1),
            Angle::new::<radian>(0.3),
        );
        let mount = Mount {
            offset: DVec2::new(0.1, 0.0),
            heading: -core::f64::consts::FRAC_PI_2,
        };
        let true_pose = Pose::new(
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.1),
            Angle::new::<radian>(0.3),
        );
        let distance = cast(true_pose, mount, &[surface]).unwrap().distance;
        let reading = RangeReading {
            distance: Length::new::<meter>(distance),
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
