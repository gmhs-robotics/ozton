# ozton

`ozton` is the Rust way of creating a playback/record auton for any vexide
robot.

1. It should record and replay routes fast and iteratively.
2. It should be assisted by GPS/other sensors.
3. `RecordableDrivetrain` wraps `Drivetrain`, so the same robot code can be
   recorded, replayed, or driven manually through motion/instruction code.
4. The derive-driven frame system should cover whole robots, including motors,
   pneumatics via `AdiDigitalOut`, and common ADI actuators.

## Credits

Highly inspired by vexide/evian and vexide/autons. A lot of source was taken
from them.
