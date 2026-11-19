# Build a programmed robot

Give `DifferentialConfig` the effective wheel diameter, track width and maximum
wheel speed once. Ozton uses that configuration for motor output, odometry,
path constraints and command settling. Choose a speed that every linked motor
can reach. Use the [calibration trials](calibration.md) to refine your measurements.

A gear ratio is wheel turns per motor output turn. A 36-tooth motor gear driving
a 48-tooth wheel gear is `GearRatio::from_teeth(36, 48)`. A direct-drive motor is
`GearRatio::DIRECT`. Use the motor's direction setting for polarity. All configured geared motors contribute to wheel feedback through encoder consensus.

## Add hardware and mechanisms

`TrackingSetup::with_imu` calibrates the supplied IMU while the builder runs.
Keep the robot still. Register a distance device and its mount together with
`with_range`. Add `with_map` when using field-surface corrections.

The stock `ozton::motion::auton::Robot::new(drive, ())` works for a chassis-only robot. For a robot with
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


## Compose a custom robot

Use `ozton::runtime::Robot` for typed hardware assembly when your application has
its own driver, tracking or mechanism policy. The builder owns the concrete
values passed to it. Unused slots default to `()` and all fields remain accessible.
There is no boxed device, boxed route future or dynamic driver dispatch.

```rust
use ozton::runtime::{DriveControl, Robot};

struct DriveBoost;

impl DriveControl<ozton::drivetrain::resilient::ResilientDrive, bool> for DriveBoost {
    fn update(
        &mut self,
        boost_pressed: &bool,
        drive: &mut ozton::drivetrain::resilient::ResilientDrive,
    ) {
        drive.set_driver_scale(if *boost_pressed { 1.0 } else { 0.75 });
    }
}

fn assemble(
    controller: vexide::controller::Controller,
    drive: ozton::drivetrain::resilient::ResilientDrive,
) -> Robot<ozton::drivetrain::resilient::ResilientDrive,
           vexide::controller::Controller, (), (), DriveBoost> {
    Robot::new()
        .with_controller(controller)
        .with_drivetrain(drive)
        .with_custom(DriveBoost)
}
```

Call `apply_custom(&input)` during the driver tick before sending outputs. The
input and drivetrain in `DriveControl` are your concrete types. A custom control
can use any method the drivetrain exposes. Append another policy with
`with_control`. Policies run in registration order, before the application
sends wheel commands. Use `with_tracking` and
`with_mechanisms` for additional device state. You can also write motor outputs
directly and omit the custom-control hook.

To reuse the competition loop with a custom application, implement
`runtime::competition::CompetitionRobot` on your application type. Keep your
driver tick and stop policy there. The runtime handles the Brain menu, mode
transitions and tick budget. Call `route_menu::<YourRobot>(display)` during
assembly and `runtime::competition::compete(robot).await`.

Register named async functions with `ozton::autonomous_routes!`. Each line adds
one Brain choice and a statically compiled route branch.

```rust
use ozton::autonomous_routes;

struct PracticeRobot;
async fn left(_robot: &mut PracticeRobot) -> Result<(), &'static str> { Ok(()) }

autonomous_routes! {
    enum Route for PracticeRobot => &'static str {
        Left("Left") => left,
    }
}
```

Use `Runner` for multi-tick autonomous commands so field disable cancels motion.
Override uses this composition with its own lift, pivot, tracking and Y-button
boost policies.
