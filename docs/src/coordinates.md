# Coordinates and units

Imagine looking down at a robot facing away from you. Ozton calls right +X and forward +Y. A heading of zero points along +Y. Positive heading turns left (counterclockwise); -90 degrees points right.

A field pose describes where the robot's chosen origin is on the field. A sensor mount describes where the sensor is relative to that origin on the robot. Mounts add a vertical Z coordinate, measured upward. Do not confuse the two frames.

## Write quantities with units

```rust
use ozton::position_xy;
use ozton::position_xyh;
use ozton::position_xyzh;

let distance = ozton::qty!(24 in);
let heading = ozton::qty!(90 deg);
let point = position_xy!(12 in, 24 in);
let pose = position_xyh!(12 in, 24 in, 90 deg);
let mount = position_xyzh!(0 in, 6 in, 4 in, 0 deg);
```

Lengths are stored in metres and angles in radians. The macros convert literals for you. You can also use uom constructors for quantities such as velocity or acceleration. A point has no heading; a pose does. Survey the robot origin, wheel diameters and sensor mounts consistently.

## Check before driving

Place the robot at a measured starting pose, align its heading, and register that pose with localization. A forward move should increase Y when heading is zero. A left turn should increase heading. Fix incorrect signs at their source instead of compensating in every route.
