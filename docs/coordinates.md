# The Origin

**The Origin** is the chassis center of rotation at floor level. It is
`(0, 0, 0)` in the robot frame and moves with the chassis. Measure each sensor
and tracking-wheel mount directly from `RobotPoint::ORIGIN`. Odometry uses the
mount coordinates to calculate wheel lever arms about that same point.

| Axis | Positive direction on the robot |
| --- | --- |
| `X` | Right |
| `Y` | Forward |
| `Z` | Up |

A mount 5 cm left, 12 cm forward, and 8 cm up has coordinates
`(-0.05, 0.12, 0.08)` in metres. Zero heading faces `+Y`. Positive heading
turns left about `+Z`, so a heading of +90° faces `-X`. Use `compass_heading`
to convert a VEX clockwise heading into this convention.

## Choose measurement axes

Use `MountAxes` if your survey labels X and Y differently. Choose perpendicular
positive directions from `Right`, `Forward`, `Left`, and `Backward`. For example,
with +X forward and +Y left, enter the same mount as follows.

```rust
use ozton::math::si;
use ozton::tracking::mount::{MountAxes, MountDirection};

const AXES: MountAxes = MountAxes::new(MountDirection::Forward, MountDirection::Left);
let mount = AXES.point(si(0.12), si(0.05), si(0.08));
```

`point` converts the lengths to the right/forward/up convention used by
tracking. Wheel travel and range beam bearings are defined by physical chassis
directions. The measurement axes apply to mount positions.

## Place the robot in the field frame

A `Pose` gives the field position of The Origin and the robot's heading.
Choose one surveyed field origin and use it for the map, route points, and
GPS registration. One option is to put the field zero at The Origin's
starting position and align the axes with the robot's starting orientation.
A centered GPS survey also works when the map and targets use that frame.
The field frame stays fixed throughout the route. Its registration heading
rotates both field axes together relative to the survey.

See [setting the starting pose](routes.md#compose-motions-and-mechanism-actions)
and [registering GPS](localization.md#register-gps).

## Use physical units

Use native `uom` types such as `ozton::uom::si::f64::Length` for physical
settings. Use `Length::new::<inch>(2.25)` for runtime conversion and
`ozton::math::si` for SI quantities in constants. Pass typed coordinates to
`Pose::from_lengths` and `RobotPoint::new`. Use `get::<meter>()` when an
interface takes a scalar in meters.
