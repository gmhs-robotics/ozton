# Build mechanisms

Use `AxisConfig::new()` for a rotary axis and `AxisConfig::linear()` for a
linear axis. Targets, feedback, and position limits use quantities for the
selected axis. Construction validates gains, voltage caps, and optional
position limits.

```rust
# extern crate ozton;
use ozton::control::{axis::AxisConfig, loops::Pid};
use ozton::uom::si::{
    angle::degree,
    electric_potential::volt,
    f64::{Angle, ElectricPotential},
};
use std::time::Duration;

let controller = AxisConfig::new()
    .with_pid(Pid::new(2.0, 0.0, 0.0, None))
    .with_maximum_voltage(ElectricPotential::new::<volt>(4.0))
    .with_position_limits(Angle::new::<degree>(0.0), Angle::new::<degree>(90.0))
    .build()
    .expect("valid axis settings")
    .with_position_tolerance(Angle::new::<degree>(0.5))
    .with_maximum_gap(Duration::from_millis(100));
```

The optional position tolerance produces zero volts for stationary targets inside
its deadband. The caller chooses how to brake at zero. Timing gaps and nonfinite targets or feedback return `None` and reset controller state. Measured
limits block outward commands while allowing travel back inside the limits.
For application-owned travel and arrival settings, `control::limits::PositionLimits`
stores both as quantities of the axis dimension. Validate static settings with
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
typed sensor sample describing motion at the output shaft. Both cap commands
to the motor's rated voltage and attempt to stop after feedback or output failures.
`stop()` resets the controller and applies the configured brake mode. Coast avoids
active holding torque against a stowed mechanical stop; loaded mechanisms may
require Hold while extended.

Implement `runtime::device::MotorDevice` for alternative or simulated motors.
Implement `VelocityMotor` only when integrated velocity feedback is available.
`PositionSensor` provides accumulated angular feedback and reference assignment;
`RotationSensor` implements it directly. Device traits expose `uom` angles,
angular velocities, and voltages.

Keep mechanism geometry, coordinated lift/pivot motion, scoring sequences, and
competition policy in the robot application.

## Progress and reference protection

`ProgressGuard` checks signed progress, maximum encoder speed and finite feedback.
Choose a rotary or linear guard to match the monitored axis. No-progress
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
