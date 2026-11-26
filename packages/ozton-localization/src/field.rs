//! Surveyed VEX V5 field perimeters in centered field coordinates.
use ozton_math::{
    geometry::{DVec2, point},
    localization::{observation::RangeSensorConfig, range::Surface},
    path::clearance::{ConvexPolygon, MAX_OBSTACLES, Obstacles},
    position_xyz,
    units::Vec3,
    uom::si::f64::Length,
};

/// A rectangular perimeter with vertical walls.
#[derive(Clone, Copy, Debug)]
pub struct Field {
    min: Vec3<Length>,
    max: Vec3<Length>,
}

impl Field {
    /// A centered rectangular field with the given interior dimensions and wall height.
    pub fn rectangle(width: Length, length: Length, wall_height: Length) -> Option<Self> {
        if ![width, length, wall_height]
            .iter()
            .all(|dimension| dimension.value.is_finite() && dimension.value > 0.0)
        {
            return None;
        }
        let right = width / 2.0;
        let front = length / 2.0;
        Some(Self {
            min: Vec3::new(-right, -front, Length::default()),
            max: Vec3::new(right, front, wall_height),
        })
    }

    /// Portable VEX field, nominal inside dimensions from Override Appendix A.
    pub const PORTABLE: Self = Self {
        min: position_xyz!(-70.20 in, -70.20 in, 0 in),
        max: position_xyz!(70.20 in, 70.20 in, 11.54 in),
    };

    /// Metal VEX field, nominal inside dimensions from Override Appendix A.
    pub const METAL: Self = Self {
        min: position_xyz!(-70.25 in, -70.25 in, 0 in),
        max: position_xyz!(70.25 in, 70.25 in, 11.50 in),
    };

    pub fn surfaces(self) -> [Surface; 4] {
        let left = self.min.right;
        let rear = self.min.forward;
        let right = self.max.right;
        let front = self.max.forward;
        [
            Surface::new(point(left, rear), point(right, rear), self.max.up),
            Surface::new(point(right, rear), point(right, front), self.max.up),
            Surface::new(point(right, front), point(left, front), self.max.up),
            Surface::new(point(left, front), point(left, rear), self.max.up),
        ]
    }

    pub fn range_config(self, mount: RangeSensorConfig) -> RangeSensorConfig {
        FieldMap::from(self).range_config(mount)
    }
}

/// Shared physical geometry API, also available to host-side motion planning.
pub use ozton_math::localization::surface::{MotionMap, Surface as MappedSurface, SurfaceMap};

impl From<SurfaceMap> for FieldMap<()> {
    fn from(map: SurfaceMap) -> Self {
        // Pure localization: do not populate the legacy motion policy arrays.
        Self {
            faces: map.faces().collect(),
            ..Self::default()
        }
    }
}

/// Motion policy attached to a physical object, not a second geometry map.
#[derive(Clone, Copy, PartialEq, Eq)]
enum ObjectPolicy {
    Fixed,
    Movable,
}

/// Identifies an object's consecutive faces in the single surveyed surface array.
struct Object {
    faces: core::ops::Range<usize>,
    policy: ObjectPolicy,
}

/// Derived, height-independent motion geometry. The containment boundary is
/// a safety policy, even when a section of the physical perimeter is open.
pub(crate) struct MotionProjection<B> {
    pub(crate) boundary: B,
    pub(crate) fixed: Obstacles,
    pub(crate) movable: Obstacles,
}

/// A validated closed rectangular containment boundary.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FieldBoundary(DVec2);

impl FieldBoundary {
    pub fn new(half_extents: DVec2) -> Option<Self> {
        (half_extents.is_finite() && half_extents.min_element() > 0.0).then_some(Self(half_extents))
    }

    pub const fn half_extents(self) -> DVec2 {
        self.0
    }
}

/// Boundary state carried from the surveyed map to the estimator.
pub trait Boundary: Copy {
    fn bounds(self) -> Option<DVec2>;
}

impl Boundary for () {
    fn bounds(self) -> Option<DVec2> {
        None
    }
}

impl Boundary for FieldBoundary {
    fn bounds(self) -> Option<DVec2> {
        Some(self.0)
    }
}

/// Surveyed 2.5D range geometry and independent virtual motion constraints.
/// Physical obstacle footprints are derived from their surveyed faces; virtual
/// obstacles never enter the range map.
#[derive(Default)]
pub struct FieldMap<B = ()> {
    faces: Vec<Surface>,
    objects: Vec<Object>,
    motion_only: Vec<ConvexPolygon>,
    ignore_movable_ranges: bool,
    boundary: B,
}

impl From<Field> for FieldMap<FieldBoundary> {
    fn from(field: Field) -> Self {
        Self {
            faces: field.surfaces().into(),
            boundary: FieldBoundary(point(field.max.right, field.max.forward)),
            objects: Vec::new(),
            motion_only: Vec::new(),
            ignore_movable_ranges: false,
        }
    }
}

impl<B: Boundary> FieldMap<B> {
    /// Configure the closed motion perimeter without creating reflecting walls.
    /// Useful when physical walls have openings or are too low for the sensors.
    pub fn with_motion_bounds(self, half_extents: DVec2) -> Option<FieldMap<FieldBoundary>> {
        Some(FieldMap {
            faces: self.faces,
            objects: self.objects,
            motion_only: self.motion_only,
            ignore_movable_ranges: self.ignore_movable_ranges,
            boundary: FieldBoundary::new(half_extents)?,
        })
    }

    /// Add one physical wall or interior face. Perimeter faces are covered by
    /// the closed containment policy; interior faces are projected as fixed obstacles.
    pub fn add(&mut self, face: Surface) -> bool {
        self.append_object(&[face], ObjectPolicy::Fixed)
    }

    fn append_object(&mut self, faces: &[Surface], policy: ObjectPolicy) -> bool {
        if self.faces.len() + faces.len() > u128::BITS as usize
            || !faces.iter().all(|face| face.is_valid())
        {
            return false;
        }
        let start = self.faces.len();
        self.faces.extend_from_slice(faces);
        self.objects.push(Object {
            faces: start..self.faces.len(),
            policy,
        });
        true
    }

    /// Register a virtual hard obstacle without adding any reflecting surfaces.
    /// It is included in path planning and predictive stopping, not MCL, EKF
    /// or distance resets. Coordinates are field-relative SI metres.
    pub fn add_motion_box(&mut self, min: DVec2, max: DVec2) -> bool {
        let Some(polygon) = ConvexPolygon::rectangle(min, max) else {
            return false;
        };
        self.append_motion_only(polygon)
    }

    /// Register a virtual convex exclusion zone, without any sensor height.
    pub fn add_motion_polygon(&mut self, vertices: &[DVec2]) -> bool {
        let Some(polygon) = ConvexPolygon::new(vertices) else {
            return false;
        };
        self.append_motion_only(polygon)
    }

    fn append_motion_only(&mut self, polygon: ConvexPolygon) -> bool {
        let Some(projection) = self.motion_projection() else {
            return false;
        };
        if projection.fixed.iter().all(Option::is_some) {
            return false;
        }
        self.motion_only.push(polygon);
        true
    }

    /// Add a fixed box once, for both localization and motion clearance.
    pub fn add_fixed_box(&mut self, min: DVec2, max: DVec2, height: Length) -> bool {
        self.add_box(min, max, height, ObjectPolicy::Fixed)
    }

    /// Add a movable box once. Its faces are localization landmarks and its XY
    /// footprint is a soft planning cost, never a hard runtime collision limit.
    pub fn add_movable_box(&mut self, min: DVec2, max: DVec2, height: Length) -> bool {
        self.add_box(min, max, height, ObjectPolicy::Movable)
    }

    fn add_box(&mut self, min: DVec2, max: DVec2, height: Length, policy: ObjectPolicy) -> bool {
        let Some(polygon) = ConvexPolygon::rectangle(min, max) else {
            return false;
        };
        self.add_polygon(polygon.vertices(), Length::default(), height, policy)
    }

    /// Register a convex polygon in the shared 2.5D map with its vertical span.
    pub fn add_fixed_polygon(&mut self, vertices: &[DVec2], bottom: Length, top: Length) -> bool {
        self.add_polygon(vertices, bottom, top, ObjectPolicy::Fixed)
    }

    pub fn add_movable_polygon(&mut self, vertices: &[DVec2], bottom: Length, top: Length) -> bool {
        self.add_polygon(vertices, bottom, top, ObjectPolicy::Movable)
    }

    fn add_polygon(
        &mut self,
        vertices: &[DVec2],
        bottom: Length,
        top: Length,
        policy: ObjectPolicy,
    ) -> bool {
        let Some(polygon) = ConvexPolygon::new(vertices) else {
            return false;
        };
        let points = polygon.vertices();
        let faces: Vec<_> = (0..points.len())
            .map(|i| {
                Surface::new(points[i], points[(i + 1) % points.len()], top - bottom)
                    .with_base_height(bottom)
            })
            .collect();
        self.append_object(&faces, policy)
    }

    /// True for a physical wall segment lying on the containment boundary.
    fn on_perimeter(&self, face: Surface) -> bool {
        let Some(bounds) = self.boundary.bounds() else {
            return false;
        };
        (0..2).any(|axis| {
            [-bounds[axis], bounds[axis]].into_iter().any(|edge| {
                (face.start[axis] - edge).abs() < 1e-9 && (face.end[axis] - edge).abs() < 1e-9
            })
        })
    }

    /// Compute motion footprints solely from the same XY endpoints raycasting
    /// uses. Reject capacity overflow rather than silently omit geometry.
    pub(crate) fn motion_projection(&self) -> Option<MotionProjection<B>> {
        let mut projection = MotionProjection {
            boundary: self.boundary,
            fixed: [None; MAX_OBSTACLES],
            movable: [None; MAX_OBSTACLES],
        };
        for object in &self.objects {
            let faces = &self.faces[object.faces.clone()];
            if faces.len() == 1 && self.on_perimeter(faces[0]) {
                continue;
            }
            let polygon = if faces.len() == 1 {
                ConvexPolygon::segment(faces[0].start, faces[0].end)?
            } else {
                let vertices: Vec<_> = faces.iter().map(|face| face.start).collect();
                ConvexPolygon::new(&vertices)?
            };
            let slots = match object.policy {
                ObjectPolicy::Fixed => &mut projection.fixed,
                ObjectPolicy::Movable => &mut projection.movable,
            };
            *slots.iter_mut().find(|slot| slot.is_none())? = Some(polygon);
        }
        for &polygon in &self.motion_only {
            *projection.fixed.iter_mut().find(|slot| slot.is_none())? = Some(polygon);
        }
        Some(projection)
    }

    /// Add a clipped square footprint as eight faces using the same height band.
    pub fn add_octagon(
        &mut self,
        center: DVec2,
        half_width: Length,
        corner: Length,
        bottom: Length,
        top: Length,
    ) -> bool {
        let edge = half_width - corner;
        let corners = [
            (-edge, -half_width),
            (edge, -half_width),
            (half_width, -edge),
            (half_width, edge),
            (edge, half_width),
            (-edge, half_width),
            (-half_width, edge),
            (-half_width, -edge),
        ];
        if !(corner.value > 0.0 && corner < half_width)
            || self.faces.len() + corners.len() > u128::BITS as usize
            || !center.is_finite()
            || !bottom.value.is_finite()
            || !top.value.is_finite()
            || bottom >= top
            || !half_width.value.is_finite()
        {
            return false;
        }
        let vertices = corners.map(|(x, y)| center + point(x, y));
        self.add_fixed_polygon(&vertices, bottom, top)
    }

    pub fn surfaces(&self) -> impl ExactSizeIterator<Item = Surface> + '_ {
        self.faces.iter().copied()
    }

    /// Use only stationary landmarks for absolute range localization, while
    /// retaining movable objects in the surveyed motion geometry.
    #[must_use]
    pub fn without_movable_range_landmarks(mut self) -> Self {
        self.ignore_movable_ranges = true;
        self
    }

    pub fn range_config(&self, mount: RangeSensorConfig) -> RangeSensorConfig {
        let mut mount = mount.with_visible_surfaces(&self.faces);
        if self.ignore_movable_ranges {
            for object in &self.objects {
                if object.policy == ObjectPolicy::Movable {
                    for index in object.faces.clone() {
                        mount.surface_mask &= !(1u128 << index);
                    }
                }
            }
        }
        mount
    }
}

#[cfg(test)]
mod tests {
    use ozton_math::position_xyzh;

    use super::*;

    #[test]
    fn movable_landmarks_can_be_excluded_without_losing_motion_geometry() {
        use ozton_math::{localization::observation::ObservationLayout, qty};

        let mut field = FieldMap::from(Field::METAL).without_movable_range_landmarks();
        assert!(field.add_movable_box(DVec2::new(-0.1, 0.4), DVec2::new(0.1, 0.6), qty!(10 in)));
        let mount = RangeSensorConfig::at_mount(position_xyzh!(0 in, 0 in, 4 in, 0 deg));
        assert_eq!(field.range_config(mount).surface_mask, 0b1111);
        assert!(field.motion_projection().unwrap().movable[0].is_some());
        let ranges = [(0, mount)];
        let localization = crate::LocalizationBuilder::new()
            .with_field(field)
            .build(&ObservationLayout {
                wheels: &[],
                ranges: &ranges,
            })
            .unwrap();
        assert!(localization.boundary().half_extents().x > 0.0);
    }

    #[test]
    fn lower_beam_hits_fixed_goal_and_upper_beam_reaches_wall() {
        use ozton_math::{geometry::Pose, localization::range::cast_masked};
        let mut field = FieldMap::from(Field::PORTABLE);
        // Exercise map entries above the former 32-surface limit.
        for offset in 0..4 {
            assert!(field.add_octagon(
                DVec2::new(0.5, 0.15 * f64::from(offset)),
                ozton_math::qty!(0.05 m),
                ozton_math::qty!(0.01 m),
                ozton_math::qty!(0 m),
                ozton_math::qty!(0.15 m)
            ));
        }
        assert!(field.add_octagon(
            DVec2::new(0.0, 0.8),
            ozton_math::qty!(0.1 m),
            ozton_math::qty!(0.02 m),
            ozton_math::qty!(0 m),
            ozton_math::qty!(0.15 m)
        ));
        let surfaces: Vec<_> = field.surfaces().collect();
        let pose = Pose::default();
        let hits = [
            position_xyzh!(0 in, 0 in, 4 in, 0 deg),
            position_xyzh!(0 in, 0 in, 8 in, 0 deg),
        ]
        .map(|mount| {
            let mount = field.range_config(RangeSensorConfig::at_mount(mount));
            cast_masked(pose, mount.mount, &surfaces, mount.surface_mask).unwrap()
        });
        assert!(hits[0].surface >= 32);
        assert!((hits[0].distance - 0.7).abs() < 1e-12);
        assert!((hits[1].distance - ozton_math::qty!(70.2 in).value).abs() < 1e-12);
    }

    #[test]
    fn low_fixed_wall_is_invisible_to_ranges_but_blocks_motion() {
        use ozton_math::{
            geometry::Pose,
            localization::{observation::ObservationLayout, range::cast_masked},
            path::clearance::{Footprint, RectangularClearance},
            qty,
        };

        let mut field = FieldMap::from(Field::METAL);
        let min = DVec2::new(-0.5, 0.4);
        let max = DVec2::new(0.5, 0.45);
        assert!(field.add_fixed_box(min, max, qty!(1 in)));
        let mount = RangeSensorConfig::at_mount(position_xyzh!(0 in, 0 in, 4 in, 0 deg));
        let low = RangeSensorConfig::at_mount(position_xyzh!(0 in, 0 in, 0.5 in, 0 deg));
        assert_eq!(field.range_config(mount).surface_mask, 0b1111);
        assert_eq!(field.range_config(low).surface_mask, 0xff);
        let map: Vec<_> = field.surfaces().collect();
        let hit = cast_masked(
            Pose::default(),
            mount.mount,
            &map,
            field.range_config(mount).surface_mask,
        )
        .unwrap();
        assert!(hit.distance > 1.0);
        let ranges = [(0, mount)];
        let localization = crate::LocalizationBuilder::new()
            .with_field(field)
            .build(&ObservationLayout {
                wheels: &[],
                ranges: &ranges,
            })
            .unwrap();
        let bounds = localization.boundary().half_extents();
        let mut clearance = RectangularClearance::new(
            Footprint::new(qty!(6 in), qty!(6 in)).unwrap(),
            bounds,
            qty!(0 in),
        )
        .unwrap();
        for polygon in localization.motion_obstacles().into_iter().flatten() {
            clearance = clearance.with_polygon(polygon).unwrap();
        }
        assert!(!clearance.contains_translation(Pose::default(), DVec2::new(0.0, 0.8)));
    }

    #[test]
    fn motion_projection_uses_same_faces_as_height_filtered_ranges() {
        use ozton_math::qty;

        let mut field = FieldMap::<()>::default()
            .with_motion_bounds(DVec2::new(1.0, 1.5))
            .unwrap();
        assert!(field.add(Surface::new(
            DVec2::new(1.0, 0.2),
            DVec2::new(1.0, 1.5),
            qty!(5.125 in),
        )));
        let fixed = (DVec2::new(-0.5, 0.2), DVec2::new(-0.4, 0.3));
        let movable = (DVec2::new(0.2, 0.2), DVec2::new(0.3, 0.3));
        assert!(field.add_fixed_box(fixed.0, fixed.1, qty!(1 in)));
        assert!(field.add_movable_box(movable.0, movable.1, qty!(10.5 in)));
        let high = RangeSensorConfig::at_mount(position_xyzh!(0 in, 0 in, 8.125 in, 0 deg));
        assert_eq!(field.range_config(high).surface_mask, 0b111100000);
        let projection = field.motion_projection().unwrap();
        assert_eq!(projection.boundary.half_extents(), DVec2::new(1.0, 1.5));
        assert_eq!(projection.fixed[0].unwrap().bounds(), fixed);
        assert_eq!(projection.movable[0].unwrap().bounds(), movable);
    }

    #[test]
    fn virtual_obstacles_are_hard_motion_constraints_without_range_faces() {
        use ozton_math::{
            geometry::Pose,
            localization::range::cast_masked,
            path::clearance::{Footprint, RectangularClearance},
            qty,
        };
        let mut field = FieldMap::from(Field::METAL);
        let sensor = RangeSensorConfig::at_mount(position_xyzh!(0 in, 0 in, 4 in, 0 deg));
        let before = field.range_config(sensor);
        let initial_faces: Vec<_> = field.surfaces().collect();
        let initial_hit = cast_masked(
            Pose::default(),
            before.mount,
            &initial_faces,
            before.surface_mask,
        )
        .unwrap();
        let min = DVec2::new(-0.5, 0.4);
        let max = DVec2::new(0.5, 0.5);
        assert!(field.add_motion_box(min, max));
        assert!(!field.add_motion_box(max, min));
        let after = field.range_config(sensor);
        let faces: Vec<_> = field.surfaces().collect();
        assert_eq!(faces.len(), initial_faces.len());
        assert_eq!(after.surface_mask, before.surface_mask);
        let hit = cast_masked(Pose::default(), after.mount, &faces, after.surface_mask).unwrap();
        assert_eq!(hit.distance, initial_hit.distance);
        let projection = field.motion_projection().unwrap();
        assert_eq!(projection.fixed[0].unwrap().bounds(), (min, max));
        let mut clearance = RectangularClearance::new(
            Footprint::new(qty!(6 in), qty!(6 in)).unwrap(),
            projection.boundary.half_extents(),
            qty!(0 in),
        )
        .unwrap();
        for polygon in projection.fixed.into_iter().flatten() {
            clearance = clearance.with_polygon(polygon).unwrap();
        }
        assert!(!clearance.contains_translation(Pose::default(), DVec2::new(0.0, 0.8)));
        assert!(clearance.contains_translation(Pose::default(), DVec2::new(0.0, -0.8)));
    }

    #[test]
    fn perimeter_models_supply_wall_spans_and_mount_height_masks() {
        let mount = RangeSensorConfig::at_mount(position_xyzh!(0 in, 0 in, 11.52 in, 0 deg));
        for (field, width, mask) in [(Field::PORTABLE, 140.40, 0b1111), (Field::METAL, 140.50, 0)] {
            let wall = field.surfaces()[0];
            assert!(
                ((wall.end - wall.start).length() - ozton_math::qty!((width) in).value).abs()
                    < 1e-12
            );
            assert_eq!(field.range_config(mount).surface_mask, mask);
        }
    }
}
