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
wall-contact start registration, convert it into `FieldMap` and explicitly
supply validated rectangular bounds with `with_motion_bounds`; bounds do not
create optical faces. Each map is flattened once when localization is built;
sensor-height masks and prepared particle rays retain the existing efficient
range implementation. At most 128 optical faces are supported.

Motion may start empty or inherit the physical map; adding a surface to
`MotionMap` does not change localization. Motion ignores the height band.
`MotionMap::clearance` supplies the existing optional host-side clearance
model, with explicitly supplied containment dimensions and footprint. It does
not enable runtime avoidance or replanning.

The previous FieldMap object-policy methods remain temporarily available for
existing callers; new code should use the separate maps.
