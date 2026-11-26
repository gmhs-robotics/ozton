//! Shared surveyed 2.5D geometry. Optical and motion maps are independent consumers.
use alloc::vec::Vec;
use glam::DVec2;
use uom::si::f64::Length;

use super::range::Surface as Face;
use crate::path::clearance::{ConvexPolygon, Footprint, RectangularClearance};

#[derive(Clone, Copy, Debug, PartialEq)]
enum Geometry {
    Wall([DVec2; 2]),
    Polygon(ConvexPolygon),
}

/// A vertical wall or a convex polygon extruded over a height band.
/// Coordinates are field-relative metres; heights are measured from the floor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Surface {
    geometry: Geometry,
    bottom: Length,
    height: Length,
}

impl Surface {
    pub fn wall(start: DVec2, end: DVec2, height: Length) -> Option<Self> {
        Self::checked(Geometry::Wall([start, end]), Length::default(), height)
    }

    pub fn polygon(vertices: &[DVec2], height: Length) -> Option<Self> {
        Self::checked(
            Geometry::Polygon(ConvexPolygon::new(vertices)?),
            Length::default(),
            height,
        )
    }

    pub fn rectangle(min: DVec2, max: DVec2, height: Length) -> Option<Self> {
        Self::checked(
            Geometry::Polygon(ConvexPolygon::rectangle(min, max)?),
            Length::default(),
            height,
        )
    }

    pub fn with_base_height(self, bottom: Length) -> Option<Self> {
        Self::checked(self.geometry, bottom, self.height)
    }

    fn checked(geometry: Geometry, bottom: Length, height: Length) -> Option<Self> {
        let surface = Self { geometry, bottom, height };
        surface.faces().all(Face::is_valid).then_some(surface)
    }

    pub fn face_count(&self) -> usize {
        match &self.geometry {
            Geometry::Wall(_) => 1,
            Geometry::Polygon(polygon) => polygon.vertices().len(),
        }
    }

    /// Optical faces retain a stable order, allowing compact per-sensor masks.
    pub fn faces(&self) -> impl Iterator<Item = Face> + '_ {
        (0..self.face_count()).map(move |i| {
            let (start, end) = match &self.geometry {
                Geometry::Wall([start, end]) => (*start, *end),
                Geometry::Polygon(polygon) => {
                    let vertices = polygon.vertices();
                    (vertices[i], vertices[(i + 1) % vertices.len()])
                }
            };
            Face::new(start, end, self.height).with_base_height(self.bottom)
        })
    }

    /// Motion ignores the height band and uses only the XY footprint.
    pub fn footprint(&self) -> Option<ConvexPolygon> {
        match self.geometry {
            Geometry::Wall([a, b]) => ConvexPolygon::segment(a, b),
            Geometry::Polygon(polygon) => Some(polygon),
        }
    }

    fn on_boundary(&self, half: DVec2) -> bool {
        let Geometry::Wall([a, b]) = self.geometry else { return false };
        (0..2).any(|axis| {
            [-half[axis], half[axis]].into_iter().any(|edge| {
                (a[axis] - edge).abs() < 1e-9 && (b[axis] - edge).abs() < 1e-9
            })
        })
    }
}

/// Empty by default. Only explicitly surveyed physical surfaces enter this map.
#[derive(Clone, Debug, Default)]
pub struct SurfaceMap {
    surfaces: Vec<Surface>,
    face_count: usize,
}

impl SurfaceMap {
    pub fn new() -> Self {
        Self::default()
    }

    /// Reject overflow atomically: the range implementation uses a 128-bit face mask.
    pub fn add(&mut self, surface: Surface) -> bool {
        let count = surface.face_count();
        if self.face_count + count > u128::BITS as usize {
            return false;
        }
        self.face_count += count;
        self.surfaces.push(surface);
        true
    }

    /// Convenience for four physical walls, not an implicit motion boundary.
    pub fn add_perimeter(&mut self, min: DVec2, max: DVec2, height: Length) -> bool {
        if !min.is_finite() || !max.is_finite() || !min.cmplt(max).all()
            || self.face_count + 4 > u128::BITS as usize
        {
            return false;
        }
        let corners = [min, DVec2::new(max.x, min.y), max, DVec2::new(min.x, max.y)];
        let mut walls = Vec::with_capacity(4);
        for i in 0..4 {
            let Some(wall) = Surface::wall(corners[i], corners[(i + 1) % 4], height) else {
                return false;
            };
            walls.push(wall);
        }
        self.surfaces.extend(walls);
        self.face_count += 4;
        true
    }

    pub fn surfaces(&self) -> &[Surface] {
        &self.surfaces
    }

    pub fn faces(&self) -> impl Iterator<Item = Face> + '_ {
        self.surfaces.iter().flat_map(|surface| surface.faces())
    }
}

/// Optional independent motion geometry. Start empty or inherit the surveyed
/// physical map, then add motion-only surfaces without changing localization.
#[derive(Clone, Debug, Default)]
pub struct MotionMap {
    surfaces: Vec<Surface>,
}

impl MotionMap {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn from_localization(map: &SurfaceMap) -> Self {
        Self { surfaces: map.surfaces.clone() }
    }

    pub fn add(&mut self, surface: Surface) {
        self.surfaces.push(surface);
    }

    pub fn surfaces(&self) -> &[Surface] {
        &self.surfaces
    }

    /// Optional existing host-side clearance model; no runtime avoidance is
    /// imposed by the map. A separately supplied containment boundary replaces
    /// coincident perimeter wall strips, avoiding duplicate constraints.
    pub fn clearance(
        &self,
        footprint: Footprint,
        half_extents: DVec2,
        margin: Length,
    ) -> Option<RectangularClearance> {
        let mut clearance = RectangularClearance::new(footprint, half_extents, margin)?;
        for surface in &self.surfaces {
            if !surface.on_boundary(half_extents) {
                clearance = clearance.with_polygon(surface.footprint()?)?;
            }
        }
        Some(clearance)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{geometry::Pose, localization::range::cast_masked, qty};

    #[test]
    fn empty_map_and_motion_only_surfaces_are_independent() {
        let mut map = SurfaceMap::new();
        assert_eq!(map.faces().count(), 0);
        assert!(map.add_perimeter(-DVec2::ONE, DVec2::ONE, qty!(1 m)));
        let mut motion = MotionMap::from_localization(&map);
        motion.add(Surface::rectangle(DVec2::splat(0.2), DVec2::splat(0.4), qty!(1 m)).unwrap());
        assert_eq!(map.faces().count(), 4);
        assert_eq!(motion.surfaces().len(), 5);
    }

    #[test]
    fn nearest_surface_depends_on_height() {
        let mut map = SurfaceMap::new();
        assert!(map.add_perimeter(-DVec2::ONE, DVec2::ONE, qty!(1 m)));
        assert!(map.add(Surface::rectangle(
            DVec2::new(-0.1, 0.4), DVec2::new(0.1, 0.6), qty!(0.2 m)
        ).unwrap()));
        let faces: Vec<_> = map.faces().collect();
        for (height, expected) in [(qty!(0.1 m), 0.4), (qty!(0.5 m), 1.0)] {
            let mask = faces.iter().enumerate().fold(0u128, |mask, (i, face)| {
                if face.visible_at(height) { mask | (1u128 << i) } else { mask }
            });
            let hit = cast_masked(Pose::default(), Pose::default(), &faces, mask).unwrap();
            assert!((hit.distance - expected).abs() < 1e-10);
        }
    }

    #[test]
    fn face_capacity_is_atomic() {
        let mut map = SurfaceMap::new();
        for _ in 0..32 {
            assert!(map.add_perimeter(-DVec2::ONE, DVec2::ONE, qty!(1 m)));
        }
        assert!(!map.add(Surface::wall(DVec2::ZERO, DVec2::X, qty!(1 m)).unwrap()));
        assert_eq!(map.faces().count(), 128);
    }
}
