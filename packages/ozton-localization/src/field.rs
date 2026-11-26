//! Optional surveyed VEX perimeter presets and physical wall-contact registration.
mod override_layout;
pub use override_layout::{Goal, GoalKind, OVERRIDE, OVERRIDE_SKILLS, OverrideLayout};
pub use ozton_math::localization::surface::{MotionMap, Surface, SurfaceMap};
use ozton_math::{geometry::DVec2, uom::si::f64::Length};

/// Optional dimensions of a centered rectangular physical perimeter.
/// A preset creates no geometry until explicitly converted into a surface map.
#[derive(Clone, Copy, Debug)]
pub struct Field {
    width: Length,
    length: Length,
    height: Length,
}

impl Field {
    pub fn rectangle(width: Length, length: Length, height: Length) -> Option<Self> {
        [width, length, height]
            .iter()
            .all(|dimension| dimension.value.is_finite() && dimension.value > 0.0)
            .then_some(Self {
                width,
                length,
                height,
            })
    }

    /// Nominal portable VEX field inside dimensions.
    pub const PORTABLE: Self = Self {
        width: ozton_math::qty!(140.40 in),
        length: ozton_math::qty!(140.40 in),
        height: ozton_math::qty!(11.54 in),
    };

    /// Nominal metal VEX field inside dimensions.
    pub const METAL: Self = Self {
        width: ozton_math::qty!(140.50 in),
        length: ozton_math::qty!(140.50 in),
        height: ozton_math::qty!(11.50 in),
    };

    pub fn surface_map(self) -> Option<SurfaceMap> {
        let mut map = SurfaceMap::new();
        let half = self.contact_bounds()?.half_extents();
        map.add_perimeter(-half, half, self.height).then_some(map)
    }

    pub fn contact_bounds(self) -> Option<FieldBoundary> {
        FieldBoundary::new(DVec2::new(self.width.value, self.length.value) * 0.5)
    }
}

/// Surveyed physical walls for optional corner and flush-contact pose registration.
/// This does not define reflecting geometry or motion constraints.
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

/// Optional physical registration bounds carried by an estimator.
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
