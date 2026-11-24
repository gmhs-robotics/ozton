# Build your first robot with Ozton

Ozton helps you write VEX V5 robot code in Rust. It runs on vexide, so you still use the familiar Motor, Controller and sensor types. Ozton adds drivetrain models, tracking, autonomous motion and a competition runtime.

You do not need a particle filter, a CAD model or even a tracking wheel to get started. Make the robot drive first. Once the motors turn the right way, add a way to measure how far it has moved. Then add position-based autonomous routines.

## Where to begin

- [Getting started](getting-started.md) walks through a complete two-motor robot.
- [Coordinates and units](coordinates.md) explains the coordinate system used throughout the API.
- [Localization](localization.md) explains how to register your sensors and estimate the robot's position.
- [Motion commands](motion.md) explains which controller to use for each task.
- [API map](api.md) points to the types and source code.

This guide describes the fix/motion-convergence development branch. Its API is not necessarily the same as a published crate or main. Pin the exact revision you have tested on your robot. Read [development](development.md) before updating competition firmware.
