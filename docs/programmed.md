# Build a programmed robot

Use `ozton::runtime::Robot` to assemble your drivetrain, controller, tracking,
mechanisms, and control state. The builder stores each component in a typed
field. Unused slots default to `()`. In the example below, the
`ConfiguredChassis` already owns the localizer, so the separate tracking slot
is unused.

```rust,ignore
{{#include ../packages/ozton/examples/programmed.rs:assemble}}
```

`with_autonomous::<Route>(display)` creates the Brain menu with **Disable**
selected. `compete()` runs driver ticks, stops outputs on mode transitions, and
dispatches the selected route once on entry to autonomous. Select routes while
disabled.

## Configure the differential chassis

Give `DifferentialConfig` the effective wheel diameter, track width, and maximum
wheel speed. Ozton uses those measurements for motor output, odometry, and
command settling. Use `config.path_limits()` to apply the same limits when
profiling routes. Choose a speed every linked motor can reach and refine the
measurements with [calibration trials](calibration.md).

A gear ratio is wheel turns per motor output turn. A 36-tooth motor gear
driving a 48-tooth wheel gear is `GearRatio::from_teeth(36, 48)`. Use
`GearRatio::DIRECT` for direct drive and the motor's direction setting for
polarity. All configured geared motors contribute to encoder consensus.
For an application that constructs motors individually, use
`DriveMotorCalibration::new(ratio, wheel_diameter)` to validate conversions
in a const setting. Attach each device with `DriveMotor::calibrated`.
`Differential::geared_arrays(left, right)` checks fixed motor counts at compile
time. Use `Differential::geared` when supplying an iterator or a model-level
wheel speed limit.

`DifferentialConfig::build` calibrates the IMU supplied through
`TrackingSetup::with_imu`. Keep the robot still during startup. Register a
distance device and its mount together with `with_range`, and add `with_map`
for field-surface corrections.

The wheel controller defaults to native VEX velocity control. With
`WheelControl::Voltage`, PID and feedforward use actual volts capped at each
motor's rating. `DifferentialConfig::anti_tip` can share the configured IMU.
Set its axis, level reading, polarity, and gain for your chassis.

## Implement control hooks

Use a type alias for the assembled robot in the hook and route signatures.
Install your application control state with `with_controls`. The last slot
is `runtime::competition::Autonomous<Route>` with just a menu, or
`Autonomous<Route, Sampler<MyReport>>` when serial diagnostics are installed.

- Implement `runtime::competition::CompetitionControls<CompetitionBot>` to
  supply the driver tick and stop all outputs. The example reads the controller,
  uses `drive_driver_arcade`, and stops the chassis and intake when the controller
  or an actuator command fails. Set `DRIVER_FAULT_POLICY` to
  `DriverFaultPolicy::KeepAvailableMotors` if healthy drive motors should continue
  after a drive-port fault in driver mode.
- Implement `motion::auton::AutonomousControls<CompetitionBot>` to give
  `Runner` access to tracking, wheel commands, and mechanism goals. Report pose
  trust from the localizer and clear retained goals on stop. Use one runner for
  each route so every command shares its deadline. See [writing routes](routes.md).
- For composable driver policies, implement `runtime::DriveControl<Drive, Input>`.
  Register the first policy with `with_custom` and append others with
  `with_control`. Call `robot.apply_custom(&input)` before sending wheel commands.
  Policies run in registration order against the concrete drivetrain.

The stock differential adapter also exposes
`ozton::motion::auton::Robot::new(drive, mechanisms)` and `compete_routes`. It
provides arcade driving and prepares registered paths from the chassis limits.
Use `()` for a chassis-only robot or implement `Mechanisms` for your mechanism
state. Use `with_driver` to map controller buttons to goals. The
[calibration guide](calibration.md) shows how to add built-in Brain trials to
this adapter.

## Complete example

Adapt the ports, chassis limits, field map, starting pose, and route before
running this example on a robot.

```rust,ignore
{{#include ../packages/ozton/examples/programmed.rs}}
```
