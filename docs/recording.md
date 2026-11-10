# Record and replay

Enable the `recording` feature with default features disabled. Derive
`RecordedRobot` for the application robot and mark non-recorded fields with
`#[record(skip)]`. Implement `Recordable::get_new_frame` to sample driver
commands. The generated frame owns how those commands are applied and how
supported outputs are stopped. A drivetrain can use open-loop recording or
tracking-corrected replay.

Start the runtime with `RecordingAutonomous::compete(robot, display)`.

1. On the Brain, select **Record new route** or **Record over** before driver
   mode. Drive the routine.
2. Disable to save the capture to SD. If the save fails, choose **Retry save**
   while disabled.
3. Select **Play** for the saved route and enter autonomous.

The capture defaults to 15 seconds. A robot can set `MAX_CAPTURE_DURATION` up
to the runtime's 60-second limit, subject to the frame and storage bounds.
An incomplete overwrite keeps the previous indexed route available. Existing
`.route` files load when their frame type matches the robot.

## Complete example

```rust,ignore
{{#include ../packages/ozton/examples/full.rs}}
```
