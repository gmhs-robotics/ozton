# API map

Start with the ozton facade crate. It re-exports the workspace modules, macros and unit types, so you rarely need to depend on each internal crate directly.

## Hardware and control

- [drivetrain](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton-drivetrain/src/lib.rs): Drive, Drivetrain, differential and other drive models, wheel output and footprint.
- [tracking](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton-tracking/src/lib.rs): Tracking, TrackingSource, encoders, IMU and distance devices.
- [localization](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton-localization/src/lib.rs): estimator types, feedback, field maps and LocalizationBuilder.
- [control](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton-math/src/control/mod.rs): PID, feedforward and mechanism control helpers.

## Motion and competition

- [motion](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton-motion/src/lib.rs): Seeking, MotionConfig and profiled path followers.
- [autonomous](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton/src/autonomous/mod.rs): shared ControlError and ControlResult for robot callbacks.
- [runtime](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton-runtime/src/lib.rs): Runtime, OztonCompete, TaskSlot, localization schedules and diagnostics.
- [ui](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton-ui/src/lib.rs): menu and route selector.

## Math and tools

- [math](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton-math/src/lib.rs): geometry, units, PID, path profiles, particle filters and observation types.
- [diagnostics](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton-diagnostics/src/lib.rs): log and profiler types.
- [examples](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton/examples): complete firmware programs.

Generate rustdoc against your own checked-out revision for exact method signatures. The source links above are useful for checking details but can change when the development branch advances.
