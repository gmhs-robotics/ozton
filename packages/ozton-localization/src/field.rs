//! Surveyed VEX V5 field perimeters in centered field coordinates.
use ozton_math::{
    geometry::{DVec2, point},
    localization::{observation::RangeSensorConfig, range::Surface},
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

type ObstacleBounds = [Option<(DVec2, DVec2)>; 4];

/// Derived, height-independent motion geometry. The containment boundary is
/// a safety policy, even when a section of the physical perimeter is open.
pub(crate) struct MotionProjection {
    pub(crate) bounds: Option<DVec2>,
    pub(crate) fixed: ObstacleBounds,
    pub(crate) movable: ObstacleBounds,
}

/// Unified 2.5D field geometry. Range masks and XY obstacle footprints are
/// derived from the same faces; object groups carry policy, not duplicate coordinates.
#[derive(Default)]
pub struct FieldMap {
    faces: Vec<Surface>,
    objects: Vec<Object>,
    motion_bounds: Option<DVec2>,
}

impl From<Field> for FieldMap {
    fn from(field: Field) -> Self {
        Self {
            faces: field.surfaces().into(),
            motion_bounds: Some(point(field.max.right, field.max.forward)),
            objects: Vec::new(),
        }
    }
}

impl FieldMap {
    /// Configure the closed motion perimeter without creating reflecting walls.
    /// Useful when physical walls have openings or are too low for the sensors.
    pub fn with_motion_bounds(mut self, half_extents: DVec2) -> Option<Self> {
        if !half_extents.is_finite() || half_extents.min_element() <= 0.0 {
            return None;
        }
        self.motion_bounds = Some(half_extents);
        Some(self)
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
        if !min.is_finite() || !max.is_finite() || !min.cmplt(max).all() {
            return false;
        }
        let corners = [min, DVec2::new(max.x, min.y), max, DVec2::new(min.x, max.y)];
        let faces = core::array::from_fn::<_, 4, _>(|index| {
            Surface::new(corners[index], corners[(index + 1) % 4], height)
        });
        self.append_object(&faces, policy)
    }

    /// True for a physical wall segment lying on the containment boundary.
    fn on_perimeter(&self, face: Surface) -> bool {
        let Some(bounds) = self.motion_bounds else {
            return false;
        };
        (0..2).any(|axis| {
            [-bounds[axis], bounds[axis]].into_iter().any(|edge| {
                (face.start[axis] - edge).abs() < 1e-9 && (face.end[axis] - edge).abs() < 1e-9
            })
        })
    }

    /// Compute motion footprints solely from the same XY endpoints raycasting
    /// uses. The current rectangular planner supports four fixed and four
    /// movable obstacles; reject overflow rather than silently omit geometry.
    pub(crate) fn motion_projection(&self) -> Option<MotionProjection> {
        let mut projection = MotionProjection {
            bounds: self.motion_bounds,
            fixed: [None; 4],
            movable: [None; 4],
        };
        for object in &self.objects {
            let faces = &self.faces[object.faces.clone()];
            if faces.len() == 1 && self.on_perimeter(faces[0]) {
                continue;
            }
            let (mut min, mut max) = faces.iter().fold(
                (DVec2::splat(f64::INFINITY), DVec2::splat(f64::NEG_INFINITY)),
                |(min, max), face| {
                    (
                        min.min(face.start).min(face.end),
                        max.max(face.start).max(face.end),
                    )
                },
            );
            // Preserve zero-thickness interior walls as conservative boxes.
            for axis in 0..2 {
                if min[axis] == max[axis] {
                    min[axis] -= 1e-9;
                    max[axis] += 1e-9;
                }
            }
            let slots = match object.policy {
                ObjectPolicy::Fixed => &mut projection.fixed,
                ObjectPolicy::Movable => &mut projection.movable,
            };
            *slots.iter_mut().find(|slot| slot.is_none())? = Some((min, max));
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
        let faces = core::array::from_fn::<_, 8, _>(|index| {
            let (x, y) = corners[index];
            let (end_x, end_y) = corners[(index + 1) % corners.len()];
            Surface::new(
                center + point(x, y),
                center + point(end_x, end_y),
                top - bottom,
            )
            .with_base_height(bottom)
        });
        self.append_object(&faces, ObjectPolicy::Fixed)
    }

    pub fn surfaces(&self) -> impl ExactSizeIterator<Item = Surface> + '_ {
        self.faces.iter().copied()
    }

    pub fn range_config(&self, mount: RangeSensorConfig) -> RangeSensorConfig {
        mount.with_visible_surfaces(&self.faces)
    }
}

#[cfg(test)]
mod tests {
    use ozton_math::position_xyzh;

    use super::*;

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
        let localization = crate::differential::LocalizationBuilder::new(field)
            .build(ObservationLayout {
                wheels: &[],
                ranges: &ranges,
            })
            .unwrap();
        let bounds = localization.rectangular_half_extents().unwrap();
        let mut clearance = RectangularClearance::new(
            Footprint::new(qty!(6 in), qty!(6 in)).unwrap(),
            bounds,
            qty!(0 in),
        )
        .unwrap();
        for (min, max) in localization.motion_obstacles().into_iter().flatten() {
            clearance = clearance.with_obstacle(min, max).unwrap();
        }
        assert!(!clearance.contains_translation(Pose::default(), DVec2::new(0.0, 0.8)));
    }

    #[test]
    fn motion_projection_uses_same_faces_as_height_filtered_ranges() {
        use ozton_math::qty;

        let mut field = FieldMap::default()
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
        assert_eq!(projection.bounds, Some(DVec2::new(1.0, 1.5)));
        assert_eq!(projection.fixed, [Some(fixed), None, None, None]);
        assert_eq!(projection.movable, [Some(movable), None, None, None]);
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
