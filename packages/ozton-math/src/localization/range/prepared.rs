//! Prepared distance-only ray casting for particle likelihoods.

use glam::DVec2;
use super::Surface;

/// Precomputed segment geometry for repeated distance-only ray queries.
/// The index in the prepared array is the original surface index.
#[derive(Clone, Copy)]
pub(in crate::localization) struct RayFace {
    start: DVec2,
    segment: DVec2,
    parallel_threshold: f64,
}

impl RayFace {
    fn new(surface: Surface) -> Self {
        let segment = surface.end - surface.start;
        let length_squared = segment.length_squared();
        Self {
            start: surface.start,
            segment,
            parallel_threshold: if length_squared > 1e-12 {
                length_squared * 1e-18
            } else {
                f64::INFINITY
            },
        }
    }

    /// Compare cross-product numerators before dividing. Rejected and
    /// non-nearest intersections need no division or segment normalization.
    #[inline]
    fn nearer(self, origin: DVec2, beam: DVec2, nearest: f64) -> Option<f64> {
        let denominator = beam.perp_dot(self.segment);
        if denominator * denominator <= self.parallel_threshold {
            return None;
        }
        let offset = self.start - origin;
        let sign = if denominator < 0.0 { -1.0 } else { 1.0 };
        let denominator = denominator.abs();
        let fraction = offset.perp_dot(beam) * sign;
        if !(0.0..=denominator).contains(&fraction) {
            return None;
        }
        let distance = offset.perp_dot(self.segment) * sign;
        if distance > 0.0 && distance <= nearest * denominator {
            let distance = distance / denominator;
            if distance.is_finite() {
                return Some(distance);
            }
        }
        None
    }
}

pub(in crate::localization) fn prepare_faces(surfaces: &[Surface], faces: &mut alloc::vec::Vec<RayFace>) {
    faces.clear();
    faces.extend(surfaces.iter().take(128).copied().map(RayFace::new));
}

pub(in crate::localization) fn prepared_ray_distance(
    origin: DVec2,
    beam: DVec2,
    faces: &[RayFace],
    mut mask: u128,
    maximum: f64,
) -> Option<f64> {
    let mut nearest = maximum;
    let mut found = false;
    while mask != 0 {
        let index = mask.trailing_zeros() as usize;
        let Some(face) = faces.get(index) else {
            break;
        };
        mask &= mask - 1;
        if let Some(distance) = face.nearer(origin, beam, nearest) {
            nearest = distance;
            found = true;
        }
    }
    found.then_some(nearest)
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
        if let Some(distance) = RayFace::new(*surface).nearer(origin, beam, nearest) {
            nearest = distance;
        }
    }
    nearest.is_finite().then_some(nearest)
}
