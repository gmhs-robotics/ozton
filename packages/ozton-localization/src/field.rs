//! Surveyed VEX V5 field perimeters in centered field coordinates.
use ozton_math::{localization::range::Surface, uom::si::f64::Length};
use ozton_tracking::{mount::Vec3, position, sources::distance::RangeSensorConfig};

/// A rectangular perimeter with vertical walls.
#[derive(Clone, Copy, Debug)]
pub struct Field {
    min: Vec3<Length>,
    max: Vec3<Length>,
}

impl Field {
    /// Portable VEX field, nominal inside dimensions from Override Appendix A.
    pub const PORTABLE: Self = Self {
        min: position!(-70.20 in, -70.20 in, 0 in),
        max: position!(70.20 in, 70.20 in, 11.54 in),
    };

    /// Metal VEX field, nominal inside dimensions from Override Appendix A.
    pub const METAL: Self = Self {
        min: position!(-70.25 in, -70.25 in, 0 in),
        max: position!(70.25 in, 70.25 in, 11.50 in),
    };

    pub fn surfaces(self) -> [Surface; 4] {
        let left = self.min.right;
        let rear = self.min.forward;
        let right = self.max.right;
        let front = self.max.forward;
        [
            Surface::new(left, rear, right, rear),
            Surface::new(right, rear, right, front),
            Surface::new(right, front, left, front),
            Surface::new(left, front, left, rear),
        ]
    }

    pub fn range_config(self, mount: RangeSensorConfig) -> RangeSensorConfig {
        FieldMap::from(self).range_config(mount)
    }
}

/// One fixed planar face, with its visible height interval above the field floor.
/// Represent curved or sloping fixtures by their surveyed cross-section bands.
#[derive(Clone, Copy, Debug)]
pub struct VerticalSurface {
    pub surface: Surface,
    pub bottom: Length,
    pub top: Length,
}

impl VerticalSurface {
    pub const fn new(surface: Surface, bottom: Length, top: Length) -> Self {
        Self {
            surface,
            bottom,
            top,
        }
    }

    fn visible_at(self, height: Length) -> bool {
        (self.bottom..self.top).contains(&height)
    }
}

/// Fixed 2.5D field geometry. Heights are resolved once per sensor at assembly.
#[derive(Default)]
pub struct FieldMap {
    faces: Vec<VerticalSurface>,
}

impl From<Field> for FieldMap {
    fn from(field: Field) -> Self {
        Self {
            faces: field
                .surfaces()
                .map(|surface| VerticalSurface::new(surface, field.min.up, field.max.up))
                .into(),
        }
    }
}

impl FieldMap {
    /// Add a surveyed surface. Leave movable game objects out of the fixed map.
    pub fn add(&mut self, face: VerticalSurface) -> bool {
        if self.faces.len() >= u128::BITS as usize
            || !face.surface.is_valid()
            || !face.bottom.value.is_finite()
            || !face.top.value.is_finite()
            || face.bottom >= face.top
        {
            return false;
        }
        self.faces.push(face);
        true
    }

    /// Add a clipped square footprint as eight faces using the same height band.
    pub fn add_octagon(
        &mut self,
        right: Length,
        forward: Length,
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
            || !right.value.is_finite()
            || !forward.value.is_finite()
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
            let face = VerticalSurface::new(
                Surface::new(right + x, forward + y, right + end_x, forward + end_y),
                bottom,
                top,
            );
            if !self.add(face) {
                self.faces.truncate(start);
                return false;
            }
        }
        true
    }

    pub fn surfaces(&self) -> impl ExactSizeIterator<Item = Surface> + '_ {
        self.faces.iter().map(|face| face.surface)
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
    use ozton_math::uom::si::{angle::degree, f64::Angle, length::inch};

    use super::*;

    #[test]
    fn lower_beam_hits_fixed_goal_and_upper_beam_reaches_wall() {
        use ozton_math::{
            geometry::Pose, localization::range::cast_masked, uom::si::length::meter,
        };

        let meters = Length::new::<meter>;
        let mut field = FieldMap::from(Field::PORTABLE);
        // Exercise map entries above the former 32-surface limit.
        for offset in 0..4 {
            assert!(field.add_octagon(
                meters(0.5),
                meters(0.15 * f64::from(offset)),
                meters(0.05),
                meters(0.01),
                meters(0.0),
                meters(0.15)
            ));
        }
        assert!(field.add_octagon(
            meters(0.0),
            meters(0.8),
            meters(0.1),
            meters(0.02),
            meters(0.0),
            meters(0.15)
        ));
        let surfaces: Vec<_> = field.surfaces().collect();
        let pose = Pose::default();
        let hits = [4, 8].map(|height| {
            let mount = field.range_config(RangeSensorConfig::at_mount(
                Vec3::new(
                    meters(0.0),
                    meters(0.0),
                    Length::new::<inch>(f64::from(height)),
                ),
                Angle::new::<degree>(0.0),
            ));
            cast_masked(pose, mount.mount, &surfaces, mount.surface_mask).unwrap()
        });
        assert!(hits[0].surface >= 32);
        assert!((hits[0].distance - 0.7).abs() < 1e-12);
        assert!((hits[1].distance - Length::new::<inch>(70.2).value).abs() < 1e-12);
    }

    #[test]
    fn perimeter_models_supply_wall_spans_and_mount_height_masks() {
        let mount =
            RangeSensorConfig::at_mount(position!(0 in, 0 in, 11.52 in), Angle::new::<degree>(0.0));
        for (field, width, mask) in [(Field::PORTABLE, 140.40, 0b1111), (Field::METAL, 140.50, 0)] {
            let wall = field.surfaces()[0];
            assert!(
                ((wall.end - wall.start).length() - Length::new::<inch>(width).value).abs() < 1e-12
            );
            assert_eq!(field.range_config(mount).surface_mask, mask);
        }
    }
}
