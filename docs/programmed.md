# Build a programmed robot

Assemble your controller, drivetrain, tracking, mechanisms, routes, and Brain
display in `main.rs`. Pass each port to its owner once. Use
`.with_mechanisms(Mechanisms { intake, lift })` for named fields, then access
them as `robot.mechanisms.lift` in either mode. Omit this step for a drive-only
robot.

```rust,ignore
{{#include ../packages/ozton/examples/programmed.rs:assemble}}
```

Select a route while disabled. `compete()` stops outputs on mode changes and runs
the selected async routine when autonomous begins.

## What happens each driver tick

1. Ozton reads one controller snapshot and refreshes lightweight inputs.
2. `Program::driver` receives the snapshot and elapsed time. It sets mechanism
   goals and returns a typed drivetrain command. Return `None` to stop the drive.
3. Ozton applies the drive command, then calls `Program::update_mechanisms`.
   Autonomous motion and waits call that same mechanism hook.
4. Ozton polls optional tracking and diagnostics within the driver tick budget.
   A controller disconnect or failed drive write stops both drive and mechanisms.

Put the driver mapping and async route functions in one program module. Keep
device logic and coordination in mechanism modules. Store physical limits in
the mechanism code so both modes enforce them. Apply driver-only speed shaping
when you build the driver command. Implement `stop_mechanisms` to bring custom
actuators to rest and discard stale goals.

## Choose a drivetrain

`DifferentialConfig::build` sets up a differential chassis with known wheel
diameter, free speed, track width, and maximum wheel speed. Its `TrackingSetup`
owns the IMU, range mounts, and field map. Keep the robot still during IMU
calibration. Use [mounted distance trackers](localization.md#add-field-references)
and enter each sensor once in wiring order.

The configured chassis supplies the feedback for `RobotMotion` automatically.
Its default controller uses native motor velocity. `WheelControl::Voltage`
uses PID and feedforward in volts, capped by the motor rating.

A different drivetrain implements `ProgramDrive` with its own driver and
autonomous command types. Use `Auton::drive_for` for bounded custom commands and
`Auton::wait_until` for mechanism predicates. Both share the selected route's
deadline and stop on competition cancellation.

## Complete example

```rust,ignore
{{#include ../packages/ozton/examples/programmed.rs}}
```
