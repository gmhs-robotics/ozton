# Ozton and Override

## Working in these repositories

- Split files approaching 200 lines into sibling modules when they contain distinct responsibilities. Keep shared algorithms and hardware conversions in one implementation.
- Commit every change with a conventional commit message. Ozton and Override are separate Git repositories.
- In Ozton, run `cargo check --workspace --all-targets --locked` and `cargo test --workspace --all-targets --locked`. Check the `logging` feature as well when changing telemetry.
- In Override, compile the `thumbv7a-vex-v5` release target. Override has no automated tests. Library tests belong in Ozton.
- The checked-in Rust toolchain and Nix flakes supply the V5 toolchain. Run `nix run .#upload` from the Override checkout to build and upload to slot 1. Append `-- --logging` for diagnostics, `-- --cold` to refresh the differential upload base, or both flags together.

## Firmware and telemetry

Initialize devices before `.compete().await`. At Override startup, keep the mechanism stowed and the chassis still while references are established, the IMU calibrates, and the distance sensors become ready. Give each device one owner.

The default firmware performs no logging. The `logging` feature emits USB diagnostics and automatically appends to `OZTON3.LOG` when the Brain has an SD card. The competition writer flushes in batches roughly once a second and retries after card errors. Decode a copied log using `python3 tools/decode_log.py /path/to/OZTON3.LOG --legacy` before using ozton-tools view-log. PROFILE rows contain inclusive per-stage timing summaries. Wheel and IMU logs help verify tuning and geometry, while clearance, polarity, loaded stability, and contact thresholds require a robot.

## Drivetrain and tracking

- Use `Drive::drive_vector` for normalized drive output. `Arcade::drive` and `Tank::drive` adapt inputs to it. `LateralDrive` marks drives that support lateral motion.
- Put the footprint on `Differential` and wheel gearing, diameter, and track width in `DifferentialTracking`. Create shared groups with `shared_motors!` and supply ratios in motor order. `DifferentialTracking::inferred` accepts measured free wheel speed.
- Initialize mounted sensors with `Tracking::new((drive_encoders, tracking_wheel, imu, range_sensor)).await`. Choose how observations contribute to pose with `LocalizationBuilder`, then assemble both with `Drivetrain::new`. Construct an ordered tuple using `DriveAssists::new` and borrow it with `AssistedDrivetrain::new` for driver control and motion. Keep `LiftSlowdownAssist` in Override’s `lift.rs`.
- Use SI quantities in new interfaces. Robot-mount coordinates are right, forward, up from the chassis origin. Field poses use right and forward. Zero heading faces forward and positive heading turns left. Use `position_xyzh!` for compile-time unit-bearing mounts.
- Estimators share timestamped observations. Missing motion axes contribute zero predicted travel, and accepted wall observations can correct those axes. The filters weight corrections by measurement or particle uncertainty. Explicit DSR uses coherent fresh observations while stationary. Use `Field::PORTABLE` or `Field::METAL` for the nominal VEX perimeter, with per-sensor height masks compiled by `FieldMap`. Model fixed goal housings as height bands. Treat movable scoring objects as transient observations. The application chooses the estimator and localization schedule; MCL only runs while the selected estimator is MCL and corrections are enabled. Register the surveyed start to align IMU heading with field coordinates.

## Routes and mechanisms

- Define an application-owned robot implementing `OztonCompete`. Register autonomous functions in `ROUTES` with `route!("Label", Self::method)` and return `robot.with_brain(display)` from the startup block. Use `Ui::custom` for a custom route selector. Override callbacks live in `src/control/`; wiring and sensor geometry live in `src/hardware.rs`.
- Register the surveyed start with `drivetrain.localization_mut().set_pose(pose)`. Pass a bare or assisted drive to `Seeking::new()` for points, field poses and stationary distance resets. Build profiled paths directly from native `Waypoint` values.
- Override's mechanisms own their local limits. Manual lift motion follows the measured lift height with the pivot. Independent `LiftPose` targets expose a wait handle. Background scoring and its actuator references remain owned by `Scoring` through mode changes. Spawn the B sequence as a task with scoring commands only. The driver callback retains exclusive chassis output.
- Put reusable algorithms in `ozton-math`, input and mechanism control laws in `ozton-math`, sensors and encoders in `ozton-tracking`, and pose estimation and field maps in `ozton-localization`. Keep drivetrain models in `ozton-drivetrain`, motion execution in `ozton-motion`, and application-owned routes and mechanism coordination in the application, competition dispatch in `ozton-runtime`, selector widgets in `ozton-ui`, and serial/SD formats in `ozton-diagnostics`.

Public documentation lives in docs/src. Keep examples and the README aligned with API changes; run `mdbook build docs` before publishing the book. The production documentation project is ozton-docs on Vercel.
