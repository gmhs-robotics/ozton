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
        let sign = if denominator < 0.0 { -1.0 } else { 1.0 };
        self.nearer_directed(origin, beam, nearest, denominator.abs(), sign)
    }

    #[inline]
    fn nearer_directed(
        self,
        origin: DVec2,
        beam: DVec2,
        nearest: f64,
        denominator: f64,
        sign: f64,
    ) -> Option<f64> {
        let offset = self.start - origin;
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

/// Eligible, nonparallel face with its beam-dependent denominator prepared once.
#[derive(Clone, Copy)]
pub(in crate::localization) struct DirectedFace {
    face: RayFace,
    denominator: f64,
    sign: f64,
}

pub(in crate::localization) fn prepare_direction(
    beam: DVec2,
    faces: &[RayFace],
    mask: u128,
    directed: &mut alloc::vec::Vec<DirectedFace>,
) {
    directed.clear();
    for index in masked_indices(mask) {
        let Some(&face) = faces.get(index) else {
            break;
        };
        let denominator = beam.perp_dot(face.segment);
        if denominator * denominator <= face.parallel_threshold {
            continue;
        }
        directed.push(DirectedFace {
            face,
            denominator: denominator.abs(),
            sign: if denominator < 0.0 { -1.0 } else { 1.0 },
        });
    }
}

pub(in crate::localization) fn directed_ray_distance(
    origin: DVec2,
    beam: DVec2,
    faces: &[DirectedFace],
    maximum: f64,
) -> Option<f64> {
    let mut nearest = maximum;
    let mut found = false;
    for face in faces {
        if let Some(distance) =
            face.face
                .nearer_directed(origin, beam, nearest, face.denominator, face.sign)
        {
            nearest = distance;
            found = true;
        }
    }
    found.then_some(nearest)
}

pub(in crate::localization) fn prepare_faces(
    surfaces: &[Surface],
    faces: &mut alloc::vec::Vec<RayFace>,
) {
    faces.clear();
    faces.extend(surfaces.iter().take(128).copied().map(RayFace::new));
}

pub(in crate::localization) fn prepared_ray_distance(
    origin: DVec2,
    beam: DVec2,
    faces: &[RayFace],
    mask: u128,
    maximum: f64,
) -> Option<f64> {
    let mut nearest = maximum;
    let mut found = false;
    for index in masked_indices(mask) {
        let Some(face) = faces.get(index) else {
            break;
        };
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
    mask: u128,
) -> Option<f64> {
    let mut nearest = f64::INFINITY;
    for index in masked_indices(mask) {
        let Some(surface) = surfaces.get(index) else {
            break;
        };
        if let Some(distance) = RayFace::new(*surface).nearer(origin, beam, nearest) {
            nearest = distance;
        }
    }
    nearest.is_finite().then_some(nearest)
}

/// Iterate the mask in native-sized words on 32-bit CPUs, preserving surface order.
/// Avoid a multiword trailing-zero search and subtraction for each visible face.
#[inline]
fn masked_indices(mask: u128) -> impl Iterator<Item = usize> {
    let words = [mask as usize, (mask >> usize::BITS) as usize];
    #[cfg(target_pointer_width = "32")]
    let words = [
        words[0],
        words[1],
        (mask >> 64) as usize,
        (mask >> 96) as usize,
    ];
    words.into_iter().enumerate().flat_map(|(word, mut bits)| {
        core::iter::from_fn(move || {
            if bits == 0 {
                return None;
            }
            let index = bits.trailing_zeros() as usize;
            bits &= bits - 1;
            Some(word * usize::BITS as usize + index)
        })
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn directed_faces_match_general_rays() {
        let surfaces = [
            Surface::new(DVec2::new(-1.0, 1.0), DVec2::new(1.0, 1.0), crate::qty!(1 m)),
            Surface::new(DVec2::new(1.0, 1.0), DVec2::new(1.0, -1.0), crate::qty!(1 m)),
            Surface::new(DVec2::new(1.0, -1.0), DVec2::new(-1.0, -1.0), crate::qty!(1 m)),
        ];
        let mut faces = alloc::vec::Vec::new();
        let mut directed = alloc::vec::Vec::new();
        prepare_faces(&surfaces, &mut faces);
        for beam in [DVec2::Y, -DVec2::Y, DVec2::X, DVec2::new(0.6, 0.8)] {
            for mask in [0, 1, 2, 5, 7, u128::MAX] {
                prepare_direction(beam, &faces, mask, &mut directed);
                for origin in [DVec2::ZERO, DVec2::new(0.3, -0.4), DVec2::new(2.0, 2.0)] {
                    assert_eq!(
                        directed_ray_distance(origin, beam, &directed, 2.0),
                        prepared_ray_distance(origin, beam, &faces, mask, 2.0)
                    );
                }
            }
        }
    }

    #[test]
    fn mask_iteration_preserves_surface_order_across_word_boundaries() {
        let indices = [0, 31, 32, 63, 64, 95, 96, 127];
        let mask = indices
            .iter()
            .fold(0_u128, |mask, index| mask | (1_u128 << index));
        assert!(masked_indices(mask).eq(indices));
        assert_eq!(masked_indices(0).next(), None);
        assert!(masked_indices(u128::MAX).eq(0..128));
    }
}
