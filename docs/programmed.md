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
2. Both the driver callback and autonomous routes use
   `async fn(robot: &mut Robot) -> Result<(), ProgramError>`. The driver reads its
   snapshot with `robot.driver_input()` and its elapsed time with
   `robot.driver_elapsed()`. Apply a chassis command with `robot.drive_driver(...)?`.
3. Ozton calls `Program::update_mechanisms` after the driver callback.
   Autonomous motion and managed waits call the same mechanism hook.
4. Ozton polls optional tracking and diagnostics within the driver tick budget.
   A controller disconnect or failed drive write stops both drive and mechanisms.

Put the driver mapping and async route functions in one program module. Keep
device logic and coordination in mechanism modules. Store physical limits in
the mechanism code so both modes enforce them. Apply driver-only speed shaping
when you build the driver command. Implement `stop_mechanisms` to bring custom
actuators to rest and discard stale goals.

## Run a route from a button

A future that borrows the whole robot holds that borrow until it completes.
For a routine that must run alongside steering, let the driver callback start
a task using a mechanism-owned command handle. It can use ordinary
`vexide::time::sleep(...).await` while the competition loop continues polling
driver input. Store the task and cancel it in the program's stop hook and
when manual inputs take over.

Return errors with `?` in both modes. Ozton reports failures to the Brain and
runs the mode's stop or release path. A route can return the program's error or
any error convertible to it.

## Choose a drivetrain

`DifferentialConfig::build` sets up a differential chassis with known wheel
diameter, free speed, track width, and maximum wheel speed. Its `TrackingSetup`
owns the IMU, range mounts, and field map. Keep the robot still during IMU
calibration. Await setup from `main` before calling `compete()`. Use [mounted distance trackers](localization.md#add-field-references)
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
