# Ozton and Override

## Working in these repositories

- Commit every change with a conventional commit message. Ozton and Override are separate Git repositories.
- In Ozton, run `cargo check --workspace --all-targets --locked` and `cargo test --workspace --all-targets --locked`. Check the `logging` feature as well when changing telemetry.
- In Override, compile the `thumbv7a-vex-v5` release target. Override has no automated tests. Library tests belong in Ozton.
- The checked-in Rust toolchain and Nix flakes supply the V5 toolchain. Run `nix run .#upload` from the Override checkout to build and upload to slot 1. Append `-- --logging` for diagnostics, `-- --cold` to refresh the differential upload base, or both flags together.

## Firmware and telemetry

Initialize devices before `.compete().await`. At Override startup, keep the mechanism stowed and the chassis still while references are established, the IMU calibrates, and the distance sensors become ready. Give each device one owner.

The default firmware performs no logging. The `logging` feature emits USB diagnostics and automatically appends to `OZTON.LOG` when the Brain has an SD card. The competition writer flushes in batches roughly once a second and retries after card errors. Use `RobotBuilder::with_diagnostics` to add periodic robot rows. Open a copied log with `cargo run -p ozton-tools -- view-log /path/to/OZTON.LOG`. Wheel and IMU logs help verify tuning and geometry, while clearance, polarity, loaded stability, and contact thresholds require a robot.

## Drivetrain and tracking

- `Differential` owns the motor sides. `Tank` sends normalized voltage, and its blanket `Arcade` implementation mixes forward and turn. `Mecanum` adds `Holonomic`. The stock models coast on stop and neutral output.
- `Drivetrain::new(model, tracking)` owns the model, tracking, and geometry. With typed wheel diameter and motor ratios, `with_wheel_geometry` calibrates encoder travel and converts physical wheel targets to motor RPM. Configure ratios in motor order. `with_wheel_feedback` can infer them from measured free wheel RPM. Use `with_track_width` for surveyed track width and `with_wheelbase` when needed.
- Attach the tracker with `with_tracking` when construction needs the calibrated drivetrain first. `TrackingBuilder` takes the IMU, mounted pods, and indexed distance sensors. Poll the pair with `DrivetrainMotion::poll_tracking` in the competition hook. The drivetrain can read heading and tilt for driver assistance. Keep limits such as lift-dependent stick scaling in the driver callback.
- Use SI quantities in new interfaces. Robot-mount coordinates are right, forward, up from the chassis origin. Field poses use right and forward. Zero heading faces forward and positive heading turns left. Use `position!` for compile-time unit-bearing mounts.
- Estimators share timestamped observations. Driver keeps odometry running and skips range corrections. DSR uses surveyed surfaces and valid, fresh packets. Restore trusted pose with an explicitly registered start or a confirmed contact correction.

## Routes and mechanisms

- Build the robot with `RobotBuilder`, and register callbacks with `route!`. Driver callbacks and route functions take only `&mut Robot` and return the same `Result` type. Override callbacks are under `src/control/`; wiring and sensor geometry are in `src/main.rs`.
- Bring `DrivetrainMotion` into scope for `set_start_pose`, `drive_path`, `try_distance_reset`, and `reset_at_goal`. The route supplies motion limits and timeouts. JerryIO exports can be parsed with `ozton_math::path::jerryio` and converted to a profiled path.
- A goal reset takes the goal center, bracket contact radius, bracket offset from the robot origin, and a route-chosen creep speed. It zeroes drive output after sustained contact or a bounded timeout. It uses registered IMU heading to locate the contact point on the goal circle. Register the start heading and approach facing the surveyed goal.
- Override's mechanisms own their local limits. Manual lift motion follows the measured lift height with the pivot. Independent `LiftPose` targets expose a wait handle. Background scoring and its actuator references remain owned by `Scoring` through mode changes. The physical B sequence and the button mapping live in the corresponding control files.
- Keep math hardware-independent. Put controller helpers in `ozton-control`, tracking in `ozton-tracking`, calibrated motion in `ozton-motion`, competition dispatch in `ozton-runtime`, and serial/SD formats in `ozton-diagnostics`. The `ozton` crate re-exports these modules.
