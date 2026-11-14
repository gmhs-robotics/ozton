# Choose a mode

Ozton runs on VEX V5 with vexide. It supplies the drivetrain, localization,
competition loop, and Brain selector. Your program supplies the wiring, chassis
measurements, and mechanism behavior.

| Mode | Use it when |
| --- | --- |
| `programmed` | You want to author routines in Rust and follow embedded PATH.JERRYIO routes. This is the default. |
| `recording` | You want to drive a route, save it to SD, and replay it in autonomous. |

For an Ozton checkout next to your robot project, use one dependency declaration.

```toml
# Programmed
ozton = { path = "../ozton/packages/ozton" }

# Recording
ozton = { path = "../ozton/packages/ozton", default-features = false, features = ["recording"] }
```

Cargo unifies dependency features. All references to `ozton` in the firmware
must agree on the mode. The [programmed guide](programmed.md) uses the stock
differential chassis. The [recording guide](recording.md) starts with
`RecordedRobot`.

## Coordinates and units

All robot-mounted coordinates originate at CC: `(0, 0, 0)` is CC, `+X` is right,
`+Y` forward and `+Z` up. `(1, 0, 0)` is one metre right of CC. Pose, sensor mounts,
tracking-wheel positions, maps, paths, MCL/EKF states, GPS and telemetry use these axes.
Heading zero faces +Y; positive angles turn left (CCW about +Z). A +90-degree heading
faces -X. `compass_heading` negates VEX clockwise degrees into this convention.
Use metres, radians, seconds and volts. Explicit inch interfaces convert once with `INCH`.

A field map needs a fixed surveyed origin, normally CC at the confirmed starting
placement, with axes aligned to that placement. Current CC's field pose changes as
it moves; device offsets remain relative to current CC. Do not recenter the map every
tick. A field-centered GPS/map is also valid when both pose and map use that same
surveyed origin and orientation. Never combine different origins without registration.

This refresh changes the previous native X-forward/Y-left API. Convert custom native
points as `new_x = -old_y`, `new_y = old_x`; robot-relative heading values remain CCW
from forward. Existing PATH.JERRYIO X-right/Y-forward exports keep their X/Y values.
Re-record old tracking-corrected recordings and review custom tracker implementations
before replay; the generic archive format does not identify an old coordinate convention.
