# Build mechanisms

Use `AxisConfig` for position or velocity control. Angles are output radians,
angular velocities are radians per second, and commands are volts. Construction
validates gains, voltage caps, and optional position limits.

```rust,ignore
use ozton::control::{axis::AxisConfig, loops::Pid};
use std::time::Duration;

let controller = AxisConfig::new()
    .with_pid(Pid::new(2.0, 0.0, 0.0, None))
    .with_maximum_voltage(4.0)
    .with_position_limits(0.0, core::f64::consts::FRAC_PI_2)
    .build()?
    .with_position_tolerance(0.01)
    .with_maximum_gap(Duration::from_millis(100));
```

The optional position tolerance produces zero volts for stationary targets inside
its deadband. The caller chooses how to brake at zero. Timing gaps, nonfinite raw
targets, and invalid feedback return `None` and reset controller state. Measured
limits block outward commands while allowing travel back inside the limits.
For application-owned travel and arrival settings, `control::limits::PositionLimits`
stores both in the chosen position unit. Validate static settings with
`const { assert!(LIMITS.valid()) }`, use `target()` to clamp a requested position,
and use `ready(position, target)` to check arrival within tolerance.

`PidMotor` adapts a motor to this controller. Its builder defaults to a 1:1 ratio
and Hold on stop. Set output revolutions per motor revolution for external gearing.
An invalid ratio returns `None`.

```rust,ignore
use ozton::runtime::motor::PidMotor;
use vexide::smart::motor::BrakeMode;

let mut pivot = PidMotor::builder(motor, controller)
    .with_output_ratio(12.0 / 48.0)
    .with_brake(BrakeMode::Coast)
    .build()?;
```

`update(target, dt)` samples integrated position and velocity and converts motor
RPM into output radians per second. `apply(sample, target, dt)` accepts an external
sensor sample already converted into output SI units. Both cap commands to the
motor's rated voltage and attempt to stop after feedback or output failures.
`stop()` resets the controller and applies the configured brake mode. Coast avoids
active holding torque against a stowed mechanical stop; loaded mechanisms may
require Hold while extended.

Implement `runtime::device::MotorDevice` for alternative or simulated motors.
Implement `VelocityMotor` only when integrated velocity feedback is available.
`PositionSensor` provides accumulated angular feedback and reference assignment;
`RotationSensor` implements it directly. Vexide-facing traits expose `Angle` and
RPM; controllers receive converted SI values. Device ownership stays concrete,
without trait objects or shared ownership requirements.

Keep mechanism geometry, coordinated lift/pivot motion, scoring sequences, and
competition policy in the robot application.

## Progress and reference protection

`ProgressGuard` checks signed progress, maximum encoder speed and finite feedback.
Use consistent units for position, minimum progress and maximum speed. No-progress
faults can rearm after neutral command; encoder jumps require explicit acknowledgment.
Do not reset the guard every tick or after a command failure. Establish a new monitor
baseline only for an explicit reference procedure or a known test-fixture teleport.

`StartupReference` preserves an established zero on reconnect. Its initializer is a
mechanism for assigning a reference, not evidence that physical home was reached.
Call it only after a limit/absolute sensor or operator confirmation proves home.
For an assembled robot, implement `ScoringControls<CompetitionBot>` on the
installed controls to supply a typed goal, arrival predicate, and timed action
callback to [`Runner::perform`](routes.md#short-commands-scoring-and-bounded-fallback).
A standalone adapter can implement `MechanismRobot` directly. Define scoring sequences, targets, geometry, gear
ratios, and collision policies in the robot application.
