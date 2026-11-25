//! Full ray hits for distance-reset geometry.

use glam::DVec2;

use super::{Hit, Surface, prepared::masked_indices};
use crate::geometry::{Pose, direction, rotation};

pub fn cast(pose: Pose, mount: Pose, surfaces: &[Surface]) -> Option<Hit> {
    cast_inner(pose, mount, surfaces, 0..surfaces.len())
}

/// Casts against eligible surfaces among the first 128 map entries. Hit indices
/// always refer to the original map, including when nearer surfaces are masked.
pub fn cast_masked(pose: Pose, mount: Pose, surfaces: &[Surface], mask: u128) -> Option<Hit> {
    cast_inner(
        pose,
        mount,
        surfaces,
        masked_indices(mask).take_while(|&index| index < surfaces.len()),
    )
}

fn cast_inner(
    pose: Pose,
    mount: Pose,
    surfaces: &[Surface],
    indices: impl Iterator<Item = usize>,
) -> Option<Hit> {
    let orientation = rotation(pose.heading);
    let origin = pose.position + orientation.rotate(mount.position);
    let beam = orientation.rotate(direction(mount.heading));
    let mut nearest = None;
    let mut nearest_distance = f64::INFINITY;
    for index in indices {
        let surface = &surfaces[index];
        let segment = surface.end - surface.start;
        let length_squared = segment.length_squared();
        let Some((distance, fraction)) =
            intersection(origin, beam, surface.start, segment, length_squared)
        else {
            continue;
        };
        if distance >= nearest_distance {
            continue;
        }
        nearest_distance = distance;
        nearest = Some((index, segment, length_squared, fraction));
    }
    let (surface, segment, length_squared, fraction) = nearest?;
    let length = crate::numeric::sqrt(length_squared);
    let normal = segment.perp() / length;
    Some(Hit {
        surface,
        distance: nearest_distance,
        edge_distance: fraction.min(1.0 - fraction) * length,
        normal,
        incidence: beam.dot(normal),
    })
}

fn intersection(
    origin: DVec2,
    beam: DVec2,
    start: DVec2,
    segment: DVec2,
    length_squared: f64,
) -> Option<(f64, f64)> {
    let denominator = beam.perp_dot(segment);
    if length_squared <= 1e-12 || denominator * denominator <= length_squared * 1e-18 {
        return None;
    }
    let offset = start - origin;
    let sign = if denominator < 0.0 { -1.0 } else { 1.0 };
    let scale = denominator.abs();
    let fraction = offset.perp_dot(beam) * sign;
    if !(0.0..=scale).contains(&fraction) {
        return None;
    }
    let distance = offset.perp_dot(segment) * sign;
    if distance <= 0.0 || !distance.is_finite() {
        return None;
    }
    let inverse = 1.0 / scale;
    let distance = distance * inverse;
    distance
        .is_finite()
        .then_some((distance, fraction * inverse))
}
