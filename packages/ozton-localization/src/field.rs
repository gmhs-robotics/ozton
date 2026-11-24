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

/// Fixed 2.5D field geometry. Heights are resolved once per sensor at assembly.
#[derive(Default)]
pub struct FieldMap {
    faces: Vec<Surface>,
    perimeter: Option<Field>,
}

impl From<Field> for FieldMap {
    fn from(field: Field) -> Self {
        Self {
            faces: field.surfaces().into(),
            perimeter: Some(field),
        }
    }
}

impl FieldMap {
    /// Half extents of an unobstructed rectangular field in right/forward order.
    pub(crate) fn rectangular_half_extents(&self) -> Option<DVec2> {
        let field = self.perimeter.filter(|_| self.faces.len() == 4)?;
        Some(point(field.max.right, field.max.forward))
    }

    /// Add a surveyed surface. Leave movable game objects out of the fixed map.
    pub fn add(&mut self, face: Surface) -> bool {
        if self.faces.len() >= u128::BITS as usize || !face.is_valid() {
            return false;
        }
        self.faces.push(face);
        true
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
        let start = self.faces.len();
        for (index, (x, y)) in corners.into_iter().enumerate() {
            let (end_x, end_y) = corners[(index + 1) % corners.len()];
            let face = Surface::new(
                center + point(x, y),
                center + point(end_x, end_y),
                top - bottom,
            )
            .with_base_height(bottom);
            if !self.add(face) {
                self.faces.truncate(start);
                return false;
            }
        }
        true
    }

    pub fn surfaces(&self) -> impl ExactSizeIterator<Item = Surface> + '_ {
        self.faces.iter().copied()
    }

    pub fn range_config(&self, mut mount: RangeSensorConfig) -> RangeSensorConfig {
        let mask = self
            .faces
            .iter()
            .enumerate()
            .fold(0, |mask, (index, face)| {
                mask | (u128::from(face.visible_at(mount.mount_height)) << index)
            });
        mount.surface_mask &= mask;
        mount
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
