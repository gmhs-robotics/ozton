# Build a programmed robot

Use `ozton::runtime::Robot` to assemble your drivetrain, controller, tracking,
custom subsystems, and control state. The builder stores each component in a typed
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
measurements with [calibration trials](calibration.md#run-a-brain-calibration-trial).

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
For a custom chassis, `DriveOptions::differential(track_width, wheel_speed)`
builds runner options from `Length` and `Velocity` values.

`DifferentialConfig::build` calibrates the IMU supplied in `TrackingSetup`.
Keep the robot still during startup. Pass map surfaces and named device/mount
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
  read the controller, drive the chassis, and return the input for this tick's
  customs. Set `DRIVER_FAULT_POLICY` to `DriverFaultPolicy::KeepAvailableMotors`
  if healthy drive motors should continue after a drive-port fault.
- Implement `motion::auton::AutonomousControls<CompetitionBot>` to provide
  tracking, wheel commands, and autonomous custom input. Use one runner for
  each route so every command shares its deadline. See [writing routes](routes.md).

## Register subsystems

Implement `runtime::Custom<Input, Error, Earlier>` on each subsystem and add it
with `.with_custom(Intake { … })`. The driver loop and autonomous runner update
the registered customs in order using the input supplied by their control hooks.
Implement `Custom::stop` to brake any outputs the subsystem owns.

A custom can use controller state or targets supplied as tick input. If a
feeder also needs the intake itself, register the intake before the feeder and
borrow it by compile-time index:

```rust,ignore
impl<Earlier> Custom<DriverInput, RobotError, Earlier> for Feeder
where
    Earlier: CustomAt<Latest, Value = Intake>,
{
    fn update(
        &mut self,
        input: &mut DriverInput,
        earlier: &mut Earlier,
    ) -> Result<(), RobotError> {
        self.feed(input, earlier.get())
    }
}

// Robot::new().with_custom(intake).with_custom(feeder)
```

`Latest` selects the immediately preceding custom. `Previous<Latest>` skips
one more. The compiler checks that the index exists and selects the expected
type when you run the robot. These indices resolve to tuple field accesses.
An independent custom can leave its `Earlier` type parameter unconstrained.

## Complete example

Adapt the ports, chassis limits, field map, starting pose, and route before
running this example on a robot.

```rust,ignore
{{#include ../packages/ozton/examples/programmed.rs}}
```
