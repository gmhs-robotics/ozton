# The Origin

**The Origin** is the robot's reference point. Choose it once for your chassis
and measure every sensor and tracking-wheel mount from it. In the robot frame,
The Origin is `(0, 0, 0)` and moves with the chassis. In code,
`RobotPoint::ORIGIN` represents that zero point.

| Axis | Positive direction on the robot |
| --- | --- |
| `X` | Right |
| `Y` | Forward |
| `Z` | Up |

A mount 5 cm left, 12 cm forward, and 8 cm up has coordinates
`(-0.05, 0.12, 0.08)` in metres. Zero heading faces `+Y`. Positive heading
turns left about `+Z`, so a heading of +90° faces `-X`. Use `compass_heading`
to convert a VEX clockwise heading into this convention.

## Place the robot in the field frame

A `Pose` gives the field position of The Origin and the robot's heading.
Choose one surveyed field origin and use it for the map, route points, and
GPS registration. One option is to put the field zero at The Origin's
starting position and align the axes with the robot's starting orientation.
A centered GPS survey also works when the map and targets use that frame.
The field frame stays fixed throughout the route.

See [setting the starting pose](routes.md#compose-motions-and-mechanism-actions)
and [registering GPS](localization.md#register-gps).

## Use physical units

Use native `uom` types such as `ozton::uom::si::f64::Length` for physical
settings. Use `Length::new::<inch>(2.25)` for runtime conversion and
`ozton::math::si` for SI quantities in constants. Pass typed coordinates to
`Pose::from_lengths` and `RobotPoint::new`. Use `get::<meter>()` when an
interface takes a scalar in meters.
