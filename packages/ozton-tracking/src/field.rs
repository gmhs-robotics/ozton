//! Surveyed VEX V5 field perimeters in centered field coordinates.
use ozton_math::{localization::range::Surface, uom::si::f64::Length};

use crate::{localizer::RangeSensorConfig, position};

/// A rectangular perimeter with vertical walls.
#[derive(Clone, Copy, Debug)]
pub struct Field {
    min: crate::mount::Vec3<Length>,
    max: crate::mount::Vec3<Length>,
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

    pub fn range_config(self, mut mount: RangeSensorConfig) -> RangeSensorConfig {
        let mask = if (self.min.up..=self.max.up).contains(&mount.mount_height) {
            0b1111
        } else {
            0
        };
        mount.surface_mask &= mask;
        mount
    }
}

#[cfg(test)]
mod tests {
    use ozton_math::uom::si::{angle::degree, f64::Angle, length::inch};

    use super::*;

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
