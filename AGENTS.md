# Ozton and Override

## Production-quality engineering (mandatory for Ozton and Override)

- Every change must be designed as a production-quality, maintainable solution. Do not commit temporary fixes, symptom-masking workarounds, ad hoc control loops, magic constants, or calibration-only substitutes for algorithms that belong in the reusable library.
- Diagnose root causes using logs, source inspection, hardware specifications, and reproducible tests. Prefer established, appropriate algorithms and idiomatic Rust. Fix defects at the owning abstraction rather than patching individual callers. Keep robot-specific configuration in Override and general algorithms in Ozton; avoid unnecessary abstractions or broad rewrites.
- Calibration must exercise the same production actuation, localization, motion-control, and safety implementations used in competition, except for deliberate, explicitly identified characterization tests. Characterize and document hardware-specific tuning; do not present provisional gains as validated.
- Preserve genuine safety invariants, competition cancellation, field containment, and hardware fault handling. Do not suppress errors or relax safeguards merely to make a test pass.
- Review changes for correctness, API consistency, latency, resource usage, and edge cases. Add focused tests for behavior and failure modes; run the relevant builds, tests, lint and formatting checks. Check CI and disclose any missing physical validation. Never claim production readiness when required verification is unavailable.
- If a root cause or necessary hardware evidence is missing, report the blocker and what evidence is needed rather than shipping a guessed workaround. Temporary diagnostic instrumentation must be clearly identified and must not replace the production solution.

## Working in these repositories

- Split files approaching 200 lines into sibling modules when they contain distinct responsibilities. Keep shared algorithms and hardware conversions in one implementation.
- Commit every change with a conventional commit message. Ozton and Override are separate Git repositories.
- In Ozton, run `cargo check --workspace --all-targets --locked` and `cargo test --workspace --all-targets --locked`. Check the `logging` feature as well when changing telemetry.
- In Override, compile the `thumbv7a-vex-v5` release target. Override has no automated tests. Library tests belong in Ozton.
- The checked-in Rust toolchain and Nix flakes supply the V5 toolchain. Run `nix run .#upload` from the Override checkout to build and upload to slot 1. Append `-- --logging` for diagnostics, `-- --cold` to refresh the differential upload base, or both flags together.

## Firmware and telemetry

Initialize devices before `.compete().await`. At Override startup, keep the mechanism stowed and the chassis still while references are established, the IMU calibrates, and the distance sensors become ready. Give each device one owner.

The default firmware performs no logging. The `logging` feature emits USB diagnostics and automatically creates a new numbered `.log` file when the Brain has an SD card. The competition writer flushes in batches roughly once a second and retries after card errors. Decode a copied log using `python3 tools/decode_log.py /path/to/1.log --legacy` before using ozton-tools view-log. PROFILE rows contain inclusive per-stage timing summaries. Wheel and IMU logs help verify tuning and geometry, while clearance, polarity, loaded stability, and contact thresholds require a robot.

## Drivetrain and tracking

- Use `Drive::drive_vector` for normalized drive output. `Drive::arcade` and `Drive::tank` adapt inputs to it. `LateralDrive` marks drives that support lateral motion.
- Put the footprint on `Differential` and wheel gearing, diameter, and track width in `DifferentialTracking`. Create shared groups with `shared_motors!` and supply ratios in motor order. `DifferentialTracking::inferred` accepts measured free wheel speed.
- Initialize mounted sensors with `Tracking::new((drive_encoders, tracking_wheel, imu, range_sensor)).await`. Choose how observations contribute to pose with `LocalizationBuilder`, then assemble both with `Drivetrain::new`. Compose assists in a tuple with `DriveAssists::new` and borrow the drive using `assists.apply_to(&mut drivetrain)`. Keep `LiftSlowdownAssist` in Override’s `lift.rs`.
- Use SI quantities in new interfaces. Robot-mount coordinates are right, forward, up from the chassis origin. Field poses use right and forward. Zero heading faces forward and positive heading turns left. Use `position_xyzh!` for compile-time unit-bearing mounts.
- Estimators share timestamped observations. Missing motion axes contribute zero predicted travel, and accepted wall observations can correct those axes. The filters weight corrections by measurement or particle uncertainty. Explicit DSR uses coherent fresh observations while stationary. Start with `SurfaceMap::new()`; optionally use `Field::PORTABLE.surface_map()` or `Field::METAL.surface_map()` for nominal perimeter geometry. Per-sensor height masks are compiled at localization startup. Model fixed goal housings as height bands. Treat movable scoring objects as transient observations. The application chooses the estimator and localization schedule; MCL only runs while the selected estimator is MCL and corrections are enabled. Register the surveyed start to align IMU heading with field coordinates.

## Routes and mechanisms

- Define an application-owned robot implementing `OztonCompete`. List autonomous routines in `ROUTES` with `route!("Label", Self::method)`. Attach a Brain UI with `Runtime::new(robot).with_ui(Ui::builder(display).build())`, or pass a custom selector factory to `with_ui`. Ozton dispatches the selected routine. Override wires hardware and defines competition routines in `src/main.rs`. The calibration route and its authored geometry live in `src/calibration.rs`.
- Configure optional motion exclusions by adding the same `Surface` type to an independent `MotionMap`, never to optical localization. Do not add a competition-specific autonomous-line exclusion until the application supplies its actual geometry. Routes are authored ahead of time. Direct motion commands never search or replan; host-side authored geometry validation is separate. Hardware and competition faults must propagate.
- Register the surveyed start with `drivetrain.localization_mut().set_pose(pose)`. Pass a bare or assisted drive to a controller capability for point and pose commands or to the stationary distance reset. Build profiled paths on the host from native `Waypoint` values and embed validated samples.
- Override's mechanisms own their local limits. Manual lift motion follows the measured lift height with the pivot. Independent `LiftPose` targets expose a wait handle. Background scoring and its actuator references remain owned by `Scoring` through mode changes. Keep the scoring sequence inside the scoring subsystem. The driver callback retains exclusive chassis output.
- Put reusable algorithms in `ozton-math`, input and mechanism control laws in `ozton-math`, sensors and encoders in `ozton-tracking`, and pose estimation and field maps in `ozton-localization`. Keep drivetrain models in `ozton-drivetrain`, optional driver control and drive assists in `ozton-assist`, motion execution in `ozton-motion`, and application-owned routes and mechanism coordination in the application, competition dispatch in `ozton-runtime`, selector widgets in `ozton-ui`, and serial/SD formats in `ozton-diagnostics`.

Public documentation lives in docs/src. Keep examples and the README aligned with API changes; run `mdbook build docs` before publishing the book. The production documentation project is ozton-docs on Vercel.
