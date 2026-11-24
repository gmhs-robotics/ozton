# Coordinates and units

Imagine looking down at a robot facing away from you. Ozton calls right +X and forward +Y. A heading of zero points along +Y. Positive heading turns left (counterclockwise); -90 degrees points right.

A field pose describes where the robot's chosen origin is on the field. A sensor mount describes where the sensor is relative to that origin on the robot. Mounts add a vertical Z coordinate, measured upward. Do not confuse the two frames.

## Write quantities with units

```rust
use ozton::{qty, position_xy, position_xyh, position_xyzh};

let distance = qty!(24 in);
let heading = qty!(90 deg);
let speed = qty!(1.2 m/s);
let acceleration = qty!(2.5 m/s^2);
let turn_acceleration = qty!(12 rad/s^2);
let voltage = qty!(6 V);
let point = position_xy!(12 in, 24 in);
let pose = position_xyh!(12 in, 24 in, 90 deg);
let mount = position_xyzh!(0 in, 6 in, 4 in, 0 deg);
```

`qty!` constructs a typed `uom` quantity, not an untyped number. Lengths are stored in metres and angles in radians. The unit notation determines the type, so passing a length to a velocity parameter is a compile-time error. All shorthand units work in constants. For a calculated value, use parentheses: `qty!((diameter / 2.0) in)`. Common units include `mm`, `cm`, `m`, `in`, `ft`, `deg`, `rad`, `m/s`, `in/s`, `m/s^2`, `rad/s`, `deg/s`, `rpm`, `rad/s^2`, `deg/s^2`, `V`, `A`, `kg`, `N`, `N*m`, `W`, and `s`. `qty!(10 ms)` returns a `uom::Time`; scheduling APIs taking `Duration` use `Duration::from_millis(10)`. Less common units can use `qty!(value, module::Quantity, unit)` or the original uom constructors. Position macros construct coordinate tuples or poses, rather than scalar quantities. A point has no heading; a pose does. Survey the robot origin, wheel diameters and sensor mounts consistently.

## Check before driving

Place the robot at a measured starting pose, align its heading, and register that pose with localization. A forward move should increase Y when heading is zero. A left turn should increase heading. Fix incorrect signs at their source instead of compensating in every route.
