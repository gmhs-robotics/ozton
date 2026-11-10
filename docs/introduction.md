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

The field origin is its center. Positive X points east, positive Y north.
A heading of zero points east and positive angles turn counterclockwise.
The robot frame has positive X forward and positive Y left. Use meters,
radians, seconds and volts. `compass_heading` converts VEX clockwise degrees
from north to Ozton radians. `INCH` converts inches to meters.
