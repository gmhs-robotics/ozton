# Build a programmed robot

Use `ozton::runtime::Robot` to assemble your drivetrain, controller, tracking,
subsystems, and control state. The builder stores each component in a typed
field. Unused slots default to `()`. In the example below, the
`ConfiguredChassis` already owns the localizer, so the separate tracking slot
is unused.

```rust,ignore
{{#include ../packages/ozton/examples/programmed.rs:assemble}}
```

`with_auton([Route::Practice]).with_ui(Ui::Brain(display))` creates the Brain menu with **Disable**
selected. `compete()` runs driver ticks, stops outputs on mode transitions, and
dispatches the selected route once on entry to autonomous. Select routes while
disabled.

## Configure the differential chassis

Pass owned motor arrays to `Differential::new(left, right)`. Mechanically
couple the wheels on each side. Gear every motor so full rated voltage
produces the same free RPM at the wheels on both sides. Full rated voltage
is 12 V for 11 W V5 motors and 8 V for 5.5 W motors. A speed mismatch makes
coupled motors fight each other. Verify wheel RPM and rotation direction on
the physical drivetrain.

For wheel feedback, pass the common wheel diameter and free wheel speed to
`with_wheel_feedback`. `DifferentialConfig` also takes track width and a
maximum commanded wheel speed. `config.path_limits()` applies the same limit
to route profiles. Every available motor contributes to encoder consensus.

`DifferentialConfig::build` calibrates the IMU supplied in `TrackingSetup`.
Keep the robot still during startup. Pass map surfaces and ordered device/mount
pairs in the same setup. The chassis exposes shared range devices through
its tracking structure for localization and calibration.

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
  read the controller, drive the chassis, and update the subsystems in one tick. Set `DRIVER_FAULT_POLICY` to `DriverFaultPolicy::KeepAvailableMotors`
  if healthy drive motors should continue after a drive-port fault.
- Implement `motion::auton::AutonomousControls<CompetitionBot>` to provide
  tracking, wheel commands, and autonomous subsystem input. Use one runner for
  each route so every command shares its deadline. See [writing routes](routes.md).

## Register subsystems

Implement `runtime::Subsystem<Input, Error>` on each subsystem and add it
with `.add_subsystem(Intake { … })`. The driver loop and autonomous runner update
the registered subsystems in order using the input supplied by their control hooks.
Implement `Subsystem::stop` to brake any outputs the subsystem owns.

When mechanisms need to coordinate, give one subsystem ownership of those
mechanisms. For example, register
`.add_subsystem(Scoring::new(intake, intake_pivot, lift))`.

## Complete example

Adapt the ports, chassis limits, field map, starting pose, and route before
running this example on a robot.

```rust,ignore
{{#include ../packages/ozton/examples/programmed.rs}}
```
