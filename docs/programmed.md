# Build a programmed robot

Give `DifferentialConfig` the effective wheel diameter, track width and maximum
wheel speed once. Ozton uses that configuration for motor output, odometry,
path constraints and command settling. Choose a speed that every linked motor
can reach. Use the [calibration trials](calibration.md) to refine your measurements.

A gear ratio is wheel turns per motor output turn. A 36-tooth motor gear driving
a 48-tooth wheel gear is `GearRatio::from_teeth(36, 48)`. A direct-drive motor is
`GearRatio::DIRECT`. Use the motor's direction setting for polarity. All configured geared motors contribute to wheel feedback through encoder consensus.

## Add hardware and mechanisms

`TrackingSetup::with_imu` calibrates the supplied IMU while the builder runs.
Keep the robot still. Register a distance device and its mount together with
`with_range`. Add `with_map` when using field-surface corrections.

The stock `ozton::motion::auton::Robot::new(drive, ())` works for a chassis-only robot. For a robot with
mechanisms, implement `Mechanisms::update` and `Mechanisms::stop`. Keep goals
in the mechanism state and clear them on stop. The stock driver loop supplies
arcade drive. Use `with_driver` to set mechanism goals from controller inputs.

The default driver fault policy stops the robot on a drive-port error.
`DriverFaultPolicy::KeepAvailableMotors` allows the healthy drive motors to
continue in driver mode. A drive fault in autonomous stops the route.

`DifferentialConfig::anti_tip` can share the configured IMU. Supply the axis,
level reading, polarity and gain measured on your chassis. The wheel controller
defaults to native VEX velocity control. With `WheelControl::Voltage`, PID and
feedforward use actual volts, capped at each motor's rating. Tune each side's
controller voltage limit for the hardware it controls.

## Complete example

This source is also built as the repository's `programmed` example. Adapt the
ports, measured limits, starting pose and route before running it on a robot.

```rust,ignore
{{#include ../packages/ozton/examples/programmed.rs}}
```

See [writing routes](routes.md) for mechanism markers and optional alignment.
For custom hardware and simulation, implement the lower-level `AutonRobot` seam
and drive it with `Runner`.


## Build a robot with custom controls

Start with Ozton's `Robot::new()`. Install the concrete hardware, driver
policies, robot controls and Brain route set in the same chain.

```rust,ignore
use ozton::runtime::Robot;

Robot::new()
    .with_controller(controller)
    .with_drivetrain(drive)
    .with_tracking(tracking)
    .with_mechanisms(mechanisms)
    .with_custom(DriveBoost)
    .with_controls(Controls::new(imu))
    .with_autonomous::<Route>(display)
    .compete()
    .await;
```

`with_autonomous::<Route>(display)` builds the menu and starts with **Disable**
selected. `compete()` owns mode transitions, stopping on transitions, route
dispatch and the driver tick budget. Each optional hardware slot defaults to
`()` and the builder stores the concrete parts directly.

Use a type alias for the builder's result in route and hook signatures.
`Controls` holds application state, such as an IMU reference or mechanism goals.
Implement `runtime::competition::CompetitionControls` for that state to supply
your driver tick and output stop. Add `motion::auton::AutonomousControls` when
using `Runner` with custom tracking and output. The runner handles deadlines,
retries and field disable.

A driver policy implements `DriveControl<Drive, Input>`. Call
`robot.apply_custom(&input)` before sending wheel commands to let the policy
modify the concrete drivetrain. Add further policies with `with_control`.
Policies run in registration order. Direct hardware access is available through
the assembled robot's fields.

Declare each route as an ordinary async function and register it with
`autonomous_routes!`. The macro supplies menu labels and a static route match.

```rust
use ozton::{autonomous_routes, runtime::Robot};

type PracticeBot = Robot<(), (), (), (), usize>;
async fn practice(robot: &mut PracticeBot) -> Result<(), &'static str> {
    robot.custom += 1;
    Ok(())
}

autonomous_routes! {
    enum Route for PracticeBot => &'static str {
        Practice("Practice") => practice,
    }
}
```
