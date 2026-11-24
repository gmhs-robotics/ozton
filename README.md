# Ozton

Ozton is a Rust library for VEX V5 robots built on vexide. Start with ordinary driver control, then add sensors and autonomous movement as your robot grows. You can use the motion algorithms without adopting a particular robot layout.

[Read the documentation](https://ozton.eli.best) · [First robot example](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton/examples/first_robot.rs) · [Full example](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton/examples/programmed.rs)

## Get started

Create a working vexide project first. The development branch uses vexide 0.9.0-beta.1 and the nightly toolchain pinned in this repository. Add Ozton to your project's dependencies:

```toml
ozton = { git = "https://github.com/gmhs-robotics/ozton", branch = "fix/motion-convergence" }
```

This is the development API. Pin a tested commit for competition firmware. The repository includes a toolchain file and V5 build configuration; follow the [getting started guide](docs/src/getting-started.md) to build the examples.

Driver control needs only a drive model. Add calibrated wheel travel and an IMU when you want reliable autonomous movement. For position-based movement, register a known starting pose before issuing commands:

```rust
use ozton::{length, angle, position_xyh, motion::PoseSeeking};

// drivetrain is an assembled, localized differential drive.
// Run inside an enabled autonomous route.
let motion = PoseSeeking::default()
    .with_position_tolerance(length!(0.75 in))
    .with_heading_tolerance(angle!(2 deg));

motion.move_to_pose(&mut drivetrain, position_xyh!(0 in, 24 in, 0 deg)).await?;
```

## What to learn next

- [Your first robot](docs/src/getting-started.md): motor setup, controller and competition modes.
- [Coordinates and units](docs/src/coordinates.md): how Ozton describes positions, headings and mounts.
- [Tracking and localization](docs/src/localization.md): wheel encoders, IMU, odometry, MCL and DSR.
- [Motion commands](docs/src/motion.md): choose between Basic, Seeking, Pure Pursuit and PoseSeeking.
- [Competition routes](docs/src/routes.md): register autonomous and driver callbacks.
- [Calibration and logs](docs/src/calibration.md): tune the robot from measured results.
- [API map](docs/src/api.md): find types, modules and examples.

Enable the `logging` feature to record detailed telemetry and profiling to `OZTON3.LOG` on a writable SD card. Decode it with `python3 tools/decode_log.py OZTON3.LOG`. See [logging and profiling](docs/src/recording.md) for the record format and interpretation.

Build the documentation locally with `mdbook build docs`, or preview changes with `mdbook serve docs --open`.

Ozton is developed in the open. Hardware validation is still necessary: CI checks the Rust workspace, simulations and V5 compilation, not how accurately your specific robot moves.

Licensed under [MIT](LICENSE-MIT.md) or [Apache 2.0](LICENSE-APACHE.md).
