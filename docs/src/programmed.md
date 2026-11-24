# Programmed autonomous

A route is an async function that sequences actions. Start with two or three small commands and add complexity after they work on the robot.

The programmed API combines drivetrain motion and mechanism goals in a single runner. RobotMotion exposes common commands, and MotionControls describes the hardware feedback and outputs for a custom platform. The stock ConfiguredChassis can be assembled with DifferentialConfig and TrackingSetup. The [complete programmed example](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton/examples/programmed.rs) shows a path loaded at startup, a registered start pose, path following and distance alignment.

## Prepared paths

Export waypoints from JerryIO and parse them using ozton::path_io::jerryio. Build a Path with physical limits before enabling autonomous, where practical. A prepared path is reusable. Keep the robot's position frame and the exported path frame consistent.

## Errors and timeouts

Give each stage a time budget. Propagate failures with ? unless you have a deliberate recovery strategy. A timed-out movement is not success. A driver or autonomous callback should never block indefinitely without yielding. Test cancellation between stages and during a movement before using a route in competition.
