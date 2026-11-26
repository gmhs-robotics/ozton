//! Nominal fixed goal housings from the 2026–2027 Override manual, Appendix A.

use ozton_math::{geometry::DVec2, qty, uom::si::f64::Length};

use super::{Field, MotionMap, Surface, SurfaceMap};

/// A VEX V5 Override match or Robot Skills field layout.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OverrideLayout {
    Match,
    Skills,
}

pub const OVERRIDE: OverrideLayout = OverrideLayout::Match;
pub const OVERRIDE_SKILLS: OverrideLayout = OverrideLayout::Skills;

/// Static Override goal types, including the alliance of low goals.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GoalKind {
    Red,
    Blue,
    Neutral,
    Midfield,
}

impl GoalKind {
    /// Nominal top of the housing, measured from the tiles.
    pub const fn height(self) -> Length {
        match self {
            Self::Red | Self::Blue => qty!(82.5 mm),
            Self::Neutral => qty!(146.5 mm),
            Self::Midfield => qty!(222.7 mm),
        }
    }

    const fn riser_height(self) -> Length {
        match self {
            Self::Red | Self::Blue => qty!(0 mm),
            Self::Neutral => qty!(64 mm),
            Self::Midfield => qty!(140.2 mm),
        }
    }
}

/// An installed goal housing in centered field coordinates, in metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Goal {
    pub center: DVec2,
    pub kind: GoalKind,
}

impl Goal {
    const fn from_drawing(x_mm: f64, y_mm: f64, kind: GoalKind) -> Self {
        const CENTER_MM: f64 = 1783.2;
        Self {
            center: DVec2::new((x_mm - CENTER_MM) * 0.001, (y_mm - CENTER_MM) * 0.001),
            kind,
        }
    }

    /// Full nominal plan-view housing envelope for motion clearance.
    pub fn footprint(self) -> Option<Surface> {
        self.band(qty!(142.5 mm), Length::default(), self.kind.height())
    }

    fn band(self, width: Length, bottom: Length, top: Length) -> Option<Surface> {
        let a = 0.5 * width.value;
        let b = a * (core::f64::consts::SQRT_2 - 1.0);
        let vertices = [
            DVec2::new(a, b),
            DVec2::new(b, a),
            DVec2::new(-b, a),
            DVec2::new(-a, b),
            DVec2::new(-a, -b),
            DVec2::new(-b, -a),
            DVec2::new(b, -a),
            DVec2::new(a, -b),
        ];
        Surface::polygon(&vertices.map(|offset| self.center + offset), top - bottom)?
            .with_base_height(bottom)
    }
}

const GOALS: [Goal; 9] = [
    Goal::from_drawing(1185.1, 2979.3, GoalKind::Neutral),
    Goal::from_drawing(2381.3, 2979.3, GoalKind::Blue),
    Goal::from_drawing(587.1, 2381.3, GoalKind::Neutral),
    Goal::from_drawing(2979.3, 2381.3, GoalKind::Blue),
    Goal::from_drawing(1783.2, 1783.2, GoalKind::Midfield),
    Goal::from_drawing(587.1, 1185.1, GoalKind::Red),
    Goal::from_drawing(2979.3, 1185.1, GoalKind::Neutral),
    Goal::from_drawing(1185.1, 587.1, GoalKind::Red),
    Goal::from_drawing(2381.3, 587.1, GoalKind::Neutral),
];

impl OverrideLayout {
    pub const fn name(self) -> &'static str {
        match self {
            Self::Match => "override-match",
            Self::Skills => "override-skills",
        }
    }

    /// Both setups install the same nine goal housings according to Appendix A.
    pub const fn goals(self) -> &'static [Goal; 9] {
        match self {
            Self::Match | Self::Skills => &GOALS,
        }
    }

    /// One full-size footprint per goal for host-side route clearance.
    /// Use the chosen field's contact bounds for containment.
    pub fn motion_map(self) -> Option<MotionMap> {
        let mut map = MotionMap::new();
        for &goal in self.goals() {
            map.add(goal.footprint()?);
        }
        Some(map)
    }

    /// Nominal perimeter and height-filtered goal faces for horizontal range sensors.
    ///
    /// Each tapered section uses the midpoint of its 142.5 mm and 88.8 mm widths.
    /// The octagonal envelope approximates housings with openings and sloped faces.
    /// Movable game objects and robots are omitted.
    pub fn surface_map(self, field: Field) -> Option<SurfaceMap> {
        let half = field.contact_bounds()?.half_extents();
        let mut map = field.surface_map()?;
        for &goal in self.goals() {
            if (goal.center.abs() + DVec2::splat(qty!(71.25 mm).value))
                .cmpge(half)
                .any()
            {
                return None;
            }
            let riser = goal.kind.riser_height();
            if riser.value > 0.0 && !map.add(goal.band(qty!(142.5 mm), Length::default(), riser)?) {
                return None;
            }
            if !map.add(goal.band(qty!(115.65 mm), riser, goal.kind.height())?) {
                return None;
            }
        }
        Some(map)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn override_layouts_keep_static_goals_and_mount_height_visibility() {
        use ozton_math::{geometry::Pose, path::clearance::Footprint};

        for layout in [OVERRIDE, OVERRIDE_SKILLS] {
            let motion = layout.motion_map().unwrap();
            assert_eq!(motion.surfaces().len(), 9);
            let chassis = Footprint::new(qty!(38 cm), qty!(38 cm)).unwrap();
            for field in [Field::PORTABLE, Field::METAL] {
                let map = layout.surface_map(field).unwrap();
                let half = field.contact_bounds().unwrap().half_extents();
                let clearance = motion.clearance(chassis, half, qty!(0 mm)).unwrap();
                for goal in layout.goals() {
                    assert!(!clearance.contains_pose(Pose::new(goal.center, Default::default())));
                }
                assert!(clearance.contains_pose(Pose::new(DVec2::splat(0.3), Default::default())));
                assert_eq!(map.faces().count(), 116);
                assert_eq!(layout.goals()[4].center, DVec2::ZERO);
                for (height, count) in [(50., 76), (92., 44), (206., 12), (240., 4)] {
                    assert_eq!(
                        map.faces()
                            .filter(|face| face.visible_at(qty!((height) mm)))
                            .count(),
                        count
                    );
                }
            }
        }
        assert_ne!(OVERRIDE.name(), OVERRIDE_SKILLS.name());
        let too_small = Field::rectangle(qty!(1 m), qty!(1 m), qty!(30 cm)).unwrap();
        assert!(OVERRIDE.surface_map(too_small).is_none());
    }
}
