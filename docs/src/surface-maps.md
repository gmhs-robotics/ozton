# Surface maps

Start with `SurfaceMap::new()`; no reflecting walls or objects exist by default. Add
`Surface::wall`, `Surface::rectangle`, or `Surface::polygon` (convex pieces for a
nonconvex object). All positions are field-relative SI metres; height is measured
from the floor. Use `with_base_height` for an elevated object.

```rust
use ozton_math::{geometry::DVec2, localization::surface::{Surface, SurfaceMap, MotionMap}, qty};
let mut map = SurfaceMap::new();
assert!(map.add_perimeter(-DVec2::ONE, DVec2::ONE, qty!(0.3 m)));
assert!(map.add(Surface::rectangle(
    DVec2::new(-0.2, 0.3), DVec2::new(0.2, 0.5), qty!(0.25 m)
).unwrap()));
let mut motion = MotionMap::from_localization(&map);
motion.add(Surface::wall(DVec2::new(-1.0, 0.0), DVec2::new(1.0, 0.0), qty!(0.1 m)).unwrap());
```

Pass the optical map to `LocalizationBuilder::with_surface_map`. For existing
wall-contact start registration, separately supply validated rectangular physical
dimensions via `with_contact_bounds`; contact bounds do not create optical
faces or motion constraints. Each map is flattened once when localization is
built; sensor-height masks and prepared particle rays retain the existing
efficient range implementation. At most 128 optical faces are supported.

Motion may start empty or inherit the physical map; adding a surface to
`MotionMap` does not change localization. Motion ignores the height band.
`MotionMap::clearance` supplies the existing optional host-side clearance
model, with explicitly supplied containment dimensions and footprint. It does
not enable runtime avoidance or replanning.

## VEX V5 Override presets

`ozton::localization::field::OVERRIDE` and `OVERRIDE_SKILLS` identify the Match
and Robot Skills layouts. Both include the same nine installed goal housings
in centered coordinates. Positive X points right, positive Y points forward,
and heading 0° points along +Y. Select the actual portable or metal perimeter.

```rust
use ozton::{
    drivetrain::Footprint,
    localization::{LocalizationBuilder, field::{Field, OVERRIDE_SKILLS}},
    qty,
};

let field = Field::PORTABLE;
let layout = OVERRIDE_SKILLS;
let bounds = field.contact_bounds().unwrap();
let tracking_builder = LocalizationBuilder::new()
    .with_surface_map(layout.surface_map(field).unwrap())
    .with_contact_bounds(bounds);
// Use surveyed chassis extents for the deployed robot.
let footprint = Footprint::new(qty!(38 cm), qty!(38 cm)).unwrap();
let path_clearance = layout.motion_map().unwrap()
    .clearance(footprint, bounds.half_extents(), qty!(1 in))
    .unwrap();
```

The optical map uses horizontal slices of the goal housings at the
sensor's measured height. It has 116 faces before height filtering. The
octagonal bands approximate tapered housings and openings. Verify clear
returns at expected ranges with the actual field and hardware. The plan-view
motion map uses full nominal goal footprints. Add independently surveyed
constraints for the robot's mechanism envelope, other installed elements,
and any route-specific restrictions.

The goal centers and heights come from the official
[field drawings](https://manuals.vex.com/v5/override/v2.1/appendix-a/).
Robot Skills starts with a different distribution of movable elements
according to [RSC4](https://manuals.vex.com/v5/override/v2.1/skills/#rsc4).
Scoring elements and other robots are excluded from the optical map.

