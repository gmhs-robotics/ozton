//! Ray casting and distance-sensor resets against known field surfaces.

use glam::DVec2;
use uom::{
    ConstZero,
    si::f64::{Length, Time},
};

use crate::geometry::Pose;
#[cfg(test)]
use crate::geometry::direction;

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
            && self.start.distance_squared(self.end) > 1e-12
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

mod cast;
mod prepared;

pub use cast::{cast, cast_masked};
pub use prepared::ray_distance;
pub(super) use prepared::{RayFace, prepare_faces, prepared_ray_distance};

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
    constrain_hit(hit, reading, gate, surfaces[hit.surface], expected_surface)
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
    constrain_hit(hit, reading, gate, surfaces[hit.surface], expected_surface)
}

fn constrain_hit(
    hit: Hit,
    reading: RangeReading,
    gate: RangeGate,
    surface: Surface,
    expected_surface: Option<usize>,
) -> Result<Constraint, Rejection> {
    if !(gate.min_range.value..=gate.max_range.value).contains(&hit.distance) {
        return Err(Rejection::NoSurface);
    }
    if expected_surface.is_some_and(|expected| expected != hit.surface) {
        return Err(Rejection::WrongSurface);
    }
    // Keep the full configured margin on long walls, but do not make
    // short landmark faces geometrically impossible to observe.
    let edge_margin = gate
        .edge_margin
        .value
        .min((surface.end - surface.start).length() * 0.125);
    if hit.edge_distance < edge_margin {
        return Err(Rejection::NearEdge);
    }
    if hit.incidence.abs() < gate.min_incidence {
        return Err(Rejection::Grazing);
    }
    let residual = reading.distance.value - hit.distance;
    let edge_reserve = hit.edge_distance - edge_margin;
    let transverse_variance = (1.0 - hit.incidence * hit.incidence).max(0.0);
    if edge_reserve * edge_reserve < residual * residual * transverse_variance {
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
    fit_details(constraints).map(|fit| (fit.change, fit.axes))
}

/// Translation fit with its actual observable direction and inlier mask.
#[derive(Debug, Clone, Copy)]
pub struct TranslationFit {
    pub change: DVec2,
    pub axes: u8,
    pub normal: Option<DVec2>,
    pub mask: u8,
}

/// Prefer all observations; if they contradict, accept a unique coherent
/// leave-one-out set of at least two beams. Ambiguous exclusions are rejected.
pub fn fit_consensus(constraints: &[(Constraint, f64)]) -> Option<TranslationFit> {
    if constraints.is_empty() || constraints.len() > 8 {
        return None;
    }
    let mask = ((1u16 << constraints.len()) - 1) as u8;
    if let Some(mut fit) = fit_details(constraints) {
        fit.mask = mask;
        return Some(fit);
    }
    if constraints.len() < 3 {
        return None;
    }
    let mut selected = None;
    let mut subset = [(Constraint::default(), 0.0); 8];
    for excluded in 0..constraints.len() {
        let mut count = 0;
        for (index, &constraint) in constraints.iter().enumerate() {
            if index != excluded {
                subset[count] = constraint;
                count += 1;
            }
        }
        if let Some(mut fit) = fit_details(&subset[..count]) {
            if selected.is_some() {
                return None;
            }
            fit.mask = mask & !(1 << excluded);
            selected = Some(fit);
        }
    }
    selected
}

fn fit_details(constraints: &[(Constraint, f64)]) -> Option<TranslationFit> {
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
    let (change, axes, normal) = if determinant > 0.05 * trace * trace {
        (
            DVec2::new(yy * rhs.x - xy * rhs.y, xx * rhs.y - xy * rhs.x) / determinant,
            2,
            None,
        )
    } else {
        // Largest eigenvalue of the symmetric 2x2 normal matrix. Select
        // the better-conditioned eigenvector without atan2 or sin/cos.
        let eigenvalue = 0.5 * (trace + libm::hypot(xx - yy, 2.0 * xy));
        let vector = if xx >= yy {
            DVec2::new(eigenvalue - yy, xy)
        } else {
            DVec2::new(xy, eigenvalue - xx)
        };
        let n = vector.normalize();
        (n * (rhs.dot(n) / eigenvalue), 1, Some(n))
    };
    if !change.is_finite() {
        return None;
    }
    // Check both each innovation and the aggregate normalized residual.
    // The latter accounts for the degrees of freedom spent fitting the pose:
    // two opposing 3-sigma residuals are not a coherent one-axis measurement.
    let mut squared_residual = 0.0;
    for (c, sigma) in constraints {
        let residual = (c.hit.normal.dot(change) - c.displacement).abs();
        let limit = (3.0 * sigma).max(0.025);
        if residual > limit {
            return None;
        }
        let normalized = residual / limit;
        squared_residual += normalized * normalized;
    }
    let degrees_of_freedom = constraints.len().saturating_sub(axes as usize);
    if degrees_of_freedom > 0 && squared_residual > degrees_of_freedom as f64 {
        return None;
    }
    Some(TranslationFit {
        change,
        axes,
        normal,
        mask: 0,
    })
}

#[cfg(test)]
mod tests {
    use uom::si::time::second as time_second;

    use super::*;

    #[test]
    fn prepared_rays_match_full_cast_across_wall_openings_and_masks() {
        let map = [
            Surface::new(
                DVec2::new(1.0, -1.0),
                DVec2::new(1.0, -0.3),
                crate::qty!(1 m),
            ),
            Surface::new(DVec2::new(1.0, 0.3), DVec2::new(1.0, 1.0), crate::qty!(1 m)),
            Surface::new(
                DVec2::new(-1.0, 1.0),
                DVec2::new(1.0, 1.0),
                crate::qty!(1 m),
            ),
            Surface::new(
                DVec2::new(-0.8, -0.5),
                DVec2::new(-0.2, 0.4),
                crate::qty!(1 m),
            ),
        ];
        let mut faces = alloc::vec::Vec::new();
        prepare_faces(&map, &mut faces);
        for x in [-0.5, 0.0, 0.5] {
            for y in [-0.7, 0.0, 0.7] {
                let origin = DVec2::new(x, y);
                for degree in (-180..180).step_by(3) {
                    let heading = crate::qty!((f64::from(degree)) deg);
                    for mask in 1..16 {
                        let expected =
                            cast_masked(Pose::new(origin, heading), Pose::default(), &map, mask)
                                .map(|hit| hit.distance);
                        let actual = prepared_ray_distance(
                            origin,
                            direction(heading),
                            &faces,
                            mask,
                            f64::INFINITY,
                        );
                        match (expected, actual) {
                            (Some(a), Some(b)) => assert!((a - b).abs() < 1e-10),
                            (None, None) => {}
                            _ => panic!("ray mismatch: {expected:?} vs {actual:?}"),
                        }
                    }
                }
            }
        }
        assert_eq!(
            prepared_ray_distance(DVec2::ZERO, DVec2::X, &faces, 3, 2.0),
            None
        );
        assert_eq!(
            prepared_ray_distance(DVec2::new(0.0, 0.5), DVec2::X, &faces, 3, 0.5),
            None
        );
    }

    #[test]
    fn consensus_rejects_one_identifiable_outlier_but_not_ambiguous_pairs() {
        let constraint = |displacement| {
            (
                Constraint {
                    hit: Hit {
                        normal: DVec2::X,
                        ..Hit::default()
                    },
                    displacement,
                    ..Constraint::default()
                },
                0.01,
            )
        };
        let readings = [constraint(0.02), constraint(0.021), constraint(0.2)];
        let fit = fit_consensus(&readings).unwrap();
        assert_eq!(fit.mask, 0b011);
        assert_eq!(fit.axes, 1);
        assert!((fit.change.x - 0.0205).abs() < 1e-12);
        assert!(fit_consensus(&[readings[0], readings[2]]).is_none());
    }

    #[test]
    fn nearest_predicted_hit_must_be_inside_sensor_interval() {
        let wall = |y| Surface::new(DVec2::new(-1.0, y), DVec2::new(1.0, y), crate::qty!(1 m));
        let reading = RangeReading {
            distance: crate::qty!(1.9 m),
            confidence: 1.0,
            age: Time::ZERO,
            occluded: false,
        };
        let constraint = |map: &[Surface]| {
            constrain_masked(
                Pose::default(),
                Pose::default(),
                reading,
                RangeGate::DEFAULT,
                map,
                None,
                u128::MAX,
            )
        };
        assert_eq!(constraint(&[wall(2.01)]), Err(Rejection::NoSurface));
        assert_eq!(
            constraint(&[wall(0.01), wall(1.9)]),
            Err(Rejection::NoSurface)
        );
        assert!(constraint(&[wall(2.0)]).is_ok());
    }

    #[test]
    fn short_landmark_face_accepts_center_but_rejects_edges() {
        let half = crate::qty!(2 in).value;
        let face = Surface::new(
            DVec2::new(-half, 0.3),
            DVec2::new(half, 0.3),
            crate::qty!(10.5 in),
        );
        let wall = Surface::new(
            DVec2::new(-1.0, 0.3),
            DVec2::new(1.0, 0.3),
            crate::qty!(5.125 in),
        );
        let reading = RangeReading {
            distance: crate::qty!(0.3 m),
            confidence: 1.0,
            age: Time::ZERO,
            occluded: false,
        };
        let check = |x, face| {
            constrain_masked(
                Pose::new(DVec2::new(x, 0.0), Default::default()),
                Pose::default(),
                reading,
                RangeGate::DEFAULT,
                &[face],
                None,
                1,
            )
        };
        assert!(check(0.0, face).is_ok());
        assert_eq!(check(half * 0.9, face), Err(Rejection::NearEdge));
        assert_eq!(check(0.95, wall), Err(Rejection::NearEdge));
    }

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
