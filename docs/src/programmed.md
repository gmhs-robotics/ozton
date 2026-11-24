# Programmed autonomous

A route is an async function that sequences actions. Start with two or three small commands and add complexity after they work on the robot.

The programmed API combines drivetrain motion and mechanism goals in a single runner. RobotMotion exposes common commands, and MotionControls describes the hardware feedback and outputs for a custom platform. The stock ConfiguredChassis can be assembled with DifferentialConfig and TrackingSetup. The [complete programmed example](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton/examples/programmed.rs) shows a path loaded at startup, a registered start pose, path following and distance alignment.

## A complete route

The following function comes directly from the programmed example. It starts at a surveyed pose, follows a prepared path and aligns using a distance sensor.

```rust,ignore
{{#include ../../packages/ozton/examples/programmed.rs:route}}
```

## Prepared paths

Define native `Waypoint` values and build a `Path` with physical limits before enabling autonomous, where practical. A prepared path is reusable. Keep the robot's position frame and the waypoint coordinate frame consistent.

```rust,ignore
{{#include ../../packages/ozton/examples/programmed.rs:prepare_path}}
```

## Errors and timeouts

Give each stage a time budget. Propagate failures with ? unless you have a deliberate recovery strategy. A timed-out movement is not success. A driver or autonomous callback should never block indefinitely without yielding. Test cancellation between stages and during a movement before using a route in competition.

A programmed path command takes one ordered marker slice: `Command::Follow { path, follower, reverse, markers }`. Use an empty slice for no mechanism actions, or a one-element slice for one action. `Command::WaitFor { ready, stable }` accepts a captured predicate and the required stable duration. The `RobotMotion::follow_path` method takes the direction and follower explicitly; `drive_to` takes the target and travel direction.
