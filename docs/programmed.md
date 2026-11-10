# Build a programmed robot

Give `DifferentialConfig` the effective wheel diameter, track width and maximum
wheel speed once. Ozton uses that configuration for motor output, odometry,
path constraints and command settling. Choose a speed that every linked motor
can reach. Use the [calibration trials](calibration.md) to refine your measurements.

A gear ratio is wheel turns per motor output turn. A 36-tooth motor gear driving
a 48-tooth wheel gear is `GearRatio::from_teeth(36, 48)`. A direct-drive motor is
`GearRatio::DIRECT`. Use the motor's direction setting for polarity. The first
configured motor on each side supplies that side's wheel feedback.

## Add hardware and mechanisms

`TrackingSetup::with_imu` calibrates the supplied IMU while the builder runs.
Keep the robot still. Register a distance device and its mount together with
`with_range`. Add `with_map` when using field-surface corrections.

`Robot::new(drive, ())` works for a chassis-only robot. For a robot with
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
