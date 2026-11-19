# Record and replay

Enable the `recording` feature with default features disabled. Derive
`RecordedRobot` for the application robot and mark non-recorded fields with
`#[record(skip)]`. Implement `Recordable::get_new_frame` to sample driver
commands. The generated frame owns how those commands are applied and how
supported outputs are stopped. Use `DifferentialVoltageFrame::arcade` to
convert throttle and turn inputs into a recorded differential command.

## Save and replay a route

Start the runtime with `RecordingAutonomous::compete(robot, display)`.

1. On the Brain, select **Record new route** or **Record over** before driver
   mode. Drive the routine.
2. Disable to save the capture to SD. If the save fails, choose **Retry save**
   while disabled.
3. Select **Play** for the saved route and enter autonomous.

The capture defaults to 15 seconds. A robot can set `MAX_CAPTURE_DURATION` up
to the runtime's 60-second limit, subject to the frame and storage bounds.
If an overwrite is interrupted, the indexed route remains available. Replay
requires a `.route` file with the robot's frame type.

## Open-loop playback

Wrap a `Drivetrain` with `RecordableDrivetrain::raw` to replay recorded wheel
voltages. This example uses `NoTracking` and records an intake motor alongside
the drivetrain.

```rust,ignore
{{#include ../packages/ozton/examples/full.rs}}
```

## GPS-corrected playback

Use `RecordableDrivetrain::with_differential_playback` and
`DifferentialPlayback::VoltageCorrection` to correct drift using GPS tracking.
Measure the GPS mounting offsets and tune `DifferentialVoltagePlayback` gains
for the chassis. To use a surveyed route origin, [register the GPS field frame](localization.md#register-gps)
when constructing the tracker. The example uses the centered GPS field frame.

```rust,ignore
{{#include ../packages/ozton/examples/gps_voltage_tracking.rs}}
```
