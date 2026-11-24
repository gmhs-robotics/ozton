//! Convex XY obstacle geometry. AABB broad phase, separating axes and
//! half-plane clipping operate on the original polygon, not its bounding box.

use glam::DVec2;

pub const MAX_POLYGON_VERTICES: usize = 16;
pub const MAX_OBSTACLES: usize = 8;
pub type Obstacles = [Option<ConvexPolygon>; MAX_OBSTACLES];

/// Counterclockwise convex polygon with cached bounds and bounded storage.
/// Construction validates convexity; nonconvex solids require convex pieces.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConvexPolygon {
    vertices: [DVec2; MAX_POLYGON_VERTICES],
    len: usize,
    min: DVec2,
    max: DVec2,
}

impl ConvexPolygon {
    pub fn new(points: &[DVec2]) -> Option<Self> {
        if !(3..=MAX_POLYGON_VERTICES).contains(&points.len())
            || !points.iter().all(|point| point.is_finite())
        {
            return None;
        }
        let n = points.len();
        let area = (0..n)
            .map(|i| points[i].perp_dot(points[(i + 1) % n]))
            .sum::<f64>();
        if !area.is_finite() || area.abs() <= 1e-14 {
            return None;
        }
        let mut vertices = [DVec2::ZERO; MAX_POLYGON_VERTICES];
        for (i, vertex) in vertices[..n].iter_mut().enumerate() {
            *vertex = points[if area > 0.0 { i } else { n - 1 - i }];
        }
        let mut min = DVec2::splat(f64::INFINITY);
        let mut max = DVec2::splat(f64::NEG_INFINITY);
        for i in 0..n {
            let a = vertices[i];
            let edge = vertices[(i + 1) % n] - a;
            if edge.length_squared() <= 1e-20
                || vertices[..n]
                    .iter()
                    .any(|&p| edge.perp_dot(p - a) < -1e-12)
            {
                return None;
            }
            min = min.min(a);
            max = max.max(a);
        }
        Some(Self {
            vertices,
            len: n,
            min,
            max,
        })
    }

    pub fn rectangle(min: DVec2, max: DVec2) -> Option<Self> {
        if !min.is_finite() || !max.is_finite() || !min.cmplt(max).all() {
            return None;
        }
        Self::new(&[min, DVec2::new(max.x, min.y), max, DVec2::new(min.x, max.y)])
    }

    /// A single reflecting interior wall is a zero-thickness line in XY;
    /// use a sub-millimetre solid strip for the convex collision representation.
    pub fn segment(a: DVec2, b: DVec2) -> Option<Self> {
        let edge = b - a;
        if !edge.is_finite() || edge.length_squared() <= 1e-20 {
            return None;
        }
        let pad = edge.perp().normalize() * 1e-6;
        Self::new(&[a - pad, b - pad, b + pad, a + pad])
    }

    pub fn vertices(&self) -> &[DVec2] {
        &self.vertices[..self.len]
    }

    pub fn bounds(&self) -> (DVec2, DVec2) {
        (self.min, self.max)
    }

    fn edges(&self) -> impl Iterator<Item = (DVec2, DVec2)> + '_ {
        (0..self.len).map(|i| {
            let a = self.vertices[i];
            (a, self.vertices[(i + 1) % self.len] - a)
        })
    }

    fn project(&self, axis: DVec2) -> (f64, f64) {
        self.vertices().iter().fold(
            (f64::INFINITY, f64::NEG_INFINITY),
            |(min, max), vertex| {
                let value = vertex.dot(axis);
                (min.min(value), max.max(value))
            },
        )
    }

    /// SAT between a convex obstacle and an oriented, translating rectangle.
    /// The swept rectangle is the Minkowski sum of the footprint and travel
    /// segment. Axes are the obstacle normals, rectangle axes and travel normal.
    pub fn intersects_sweep(
        &self,
        center: DVec2,
        right: DVec2,
        half: DVec2,
        travel: DVec2,
        margin: f64,
    ) -> bool {
        let forward = right.perp();
        let extent = right.abs() * half.x + forward.abs() * half.y;
        let lo = center.min(center + travel) - extent - DVec2::splat(margin);
        let hi = center.max(center + travel) + extent + DVec2::splat(margin);
        if hi.cmplt(self.min).any() || lo.cmpgt(self.max).any() {
            return false;
        }
        let separated = |axis: DVec2| {
            let (a, b) = self.project(axis);
            let radius = half.x * axis.dot(right).abs()
                + half.y * axis.dot(forward).abs()
                + margin * axis.length();
            let start = center.dot(axis);
            let end = start + travel.dot(axis);
            start.max(end) + radius < a || start.min(end) - radius > b
        };
        if separated(right) || separated(forward) {
            return false;
        }
        if travel.length_squared() > 1e-20 && separated(travel.perp()) {
            return false;
        }
        // Outward polygon half-plane normals: only the lower support of the
        // swept box is needed, since all polygon vertices lie behind each edge.
        !self.edges().any(|(a, edge)| {
            let axis = -edge.perp();
            let radius = half.x * axis.dot(right).abs()
                + half.y * axis.dot(forward).abs()
                + margin * axis.length();
            let start = (center - a).dot(axis);
            start.min(start + travel.dot(axis)) > radius
        })
    }

    pub fn intersects_aabb(&self, min: DVec2, max: DVec2, margin: f64) -> bool {
        self.intersects_sweep((min + max) * 0.5, DVec2::X, (max - min) * 0.5, DVec2::ZERO, margin)
    }

    /// Exact circle/polygon distance test, including the interior.
    pub fn intersects_circle(&self, center: DVec2, radius: f64) -> bool {
        if center.distance_squared(center.clamp(self.min, self.max)) > radius * radius {
            return false;
        }
        let mut inside = true;
        for (a, edge) in self.edges() {
            let offset = center - a;
            inside &= edge.perp_dot(offset) >= 0.0;
            let t = (offset.dot(edge) / edge.length_squared()).clamp(0.0, 1.0);
            if offset.distance_squared(edge * t) <= radius * radius {
                return true;
            }
        }
        inside
    }

    /// Cyrus-Beck clipping against the offset supporting half planes.
    /// An offset polygon conservatively contains the Euclidean disk dilation.
    pub fn clip_segment(&self, a: DVec2, b: DVec2, padding: f64) -> Option<(f64, f64)> {
        let delta = b - a;
        let mut enter: f64 = 0.0;
        let mut exit: f64 = 1.0;
        for (start, edge) in self.edges() {
            let normal = -edge.perp();
            let numerator = padding * edge.length() - normal.dot(a - start);
            let denominator = normal.dot(delta);
            if denominator.abs() <= 1e-14 {
                if numerator < 0.0 {
                    return None;
                }
            } else if denominator < 0.0 {
                enter = enter.max(numerator / denominator);
            } else {
                exit = exit.min(numerator / denominator);
            }
            if enter > exit {
                return None;
            }
        }
        Some((enter, exit))
    }

    /// Vertex of the intersection of adjacent offset supporting lines.
    pub fn offset_vertex(&self, index: usize, padding: f64) -> Option<DVec2> {
        let point = *self.vertices().get(index)?;
        let prev = self.vertices[(index + self.len - 1) % self.len];
        let next = self.vertices[(index + 1) % self.len];
        let a = -(point - prev).perp().normalize();
        let b = -(next - point).perp().normalize();
        let denominator = 1.0 + a.dot(b);
        (denominator > 1e-12).then_some(point + (a + b) * (padding / denominator))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn triangle_does_not_block_the_empty_corner_of_its_aabb() {
        let triangle = ConvexPolygon::new(&[DVec2::ZERO, DVec2::X, DVec2::Y]).unwrap();
        assert!(!triangle.intersects_aabb(DVec2::splat(0.8), DVec2::splat(0.9), 0.0));
        assert!(triangle.intersects_aabb(DVec2::splat(0.4), DVec2::splat(0.6), 0.0));
        assert!(triangle.clip_segment(DVec2::new(-0.1, 0.2), DVec2::new(1.1, 0.2), 0.0).is_some());
        assert!(triangle.clip_segment(DVec2::new(0.8, 0.8), DVec2::ONE, 0.0).is_none());
    }
}
