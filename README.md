# Ozton

Rust drivetrain, tracking, and competition control for VEX V5, built with vexide.

See [AGENTS.md](AGENTS.md) for development, deployment, and route documentation.
The [programmed example](packages/ozton/examples/programmed.rs) shows a complete robot.

## Tracking

Use a flat tuple to register sensors in sampling order.

```rust
let tracking = Tracking::new((drive_encoders, tracking_wheel, imu, front_range)).await?;
let localization = LocalizationBuilder::default().build(tracking.observation_layout());
```

Use `shared_motors!` to create the motor groups, pass them to
`Differential::from_shared` and `DifferentialTracking::from_shared`.
The gear-ratio arrays have one entry per motor, checked at compile time.

Custom devices implement `TrackingSource::sample` and can emit any combination
of timestamped `Measurement` values. The differential estimator uses wheel
travel, heading, and range observations. Absolute pose measurements are
available to custom estimators.

Assemble the drive with `Drivetrain::new(model, tracking, localization)`, then
call `RobotBuilder::with_localization_schedule(LocalizationSchedule::Autonomous)`
after installing it. `Always` also updates during driver control and while
disabled. A drivetrain can use `()` for either component it does not need.

## Autonomous motion

Driver control and motion share the `Drive` interface. `Arcade::drive` accepts
forward and turn commands, `Tank::drive` accepts left and right commands, and
`LateralDrive` marks drives that support lateral motion.

Install an ordered tuple of assists with
`RobotBuilder::with_drive_assists(DriveAssists::new((assist_a, assist_b)))`.
Borrow the drive and assists together in driver control.

```rust
let mut drive = AssistedDrivetrain::new(&mut robot.drivetrain, &mut robot.assists);
Arcade::drive(&mut drive, forward, turn)?;
```

Pass a mutable drive to each motion. For a calibrated differential drive
with Ozton's live field localization, `PoseSeeking` plans clearance from walls,
settles with wheel and heading feedback, and stops on disable or timeout.

```rust
use ozton::{angle, length, position_xy, position_xyh};

let seeking = PoseSeeking::default()
    .with_position_tolerance(length!(0.75 in))
    .with_heading_tolerance(angle!(2 deg));

seeking.move_to_pose(&mut drivetrain, position_xyh!(0 m, 0 m, 0 deg)).await?;
seeking.reverse().move_to_point(&mut drivetrain, position_xy!(45 cm, 60 cm)).await?;
```

Use `with_tuning(DriveTuning { linear, angular })` to supply both PIDs for a
chassis. The outer loops command metres per second and radians per second.
`with_wheel_speed` limits the wheels. `with_timeout` sets a per-stage timeout.
Use `without_timeout` to let field control end an unfinished motion.

`Seeking`, `Basic`, and `PurePursuit` work with a bare or assisted drive
that provides the feedback each motion needs. `ozton::tracking::WheeledTracking`
is a standalone wheeled odometry adapter with its own update task. Point
coordinates are right and forward in metres. Use `position_xy!` for
unit-bearing point literals and `position_xyzh!` for sensor mounts.

```rust
let seeking = Seeking::default();
let outcome = seeking
    .move_to_point(&mut drivetrain, ozton::position_xy!(24 in, 24 in))
    .reverse()
    .with_angular_kp(0.4)
    .without_timeout()
    .with_linear_output_limit(0.75)
    .await?;

let basic = Basic::default();
let outcome = basic.drive_distance(&mut drivetrain, ozton::length!(24 in)).await?;
```

The generic motions return `MotionOutcome::Settled` or `MotionOutcome::TimedOut`.
`with_linear_output_limit(0.75)` sets the same throttle cap as the voltage form.
For a custom motion, write an async function that takes `&mut impl Drive`. Add
`HasLocalization` and the relevant tracking bounds when it needs feedback.

## Units and portable crates

`ozton::units` provides `length!(24 in)`, `angle!(90 deg)`, and named constructors
such as `velocity!(0.5, meter_per_second)` for the uom SI quantities. The four
position macros match the data expected by motion and mounting APIs.

| Macro | Result |
| --- | --- |
| `position_xy!(24 in, 60 cm)` | Point in metres |
| `position_xyh!(24 in, 60 cm, 90 deg)` | Planar pose |
| `position_xyz!(2 in, -4 cm, 25 mm)` | Three typed lengths |
| `position_xyzh!(2 in, -4 cm, 25 mm, 90 deg)` | Three typed lengths and a heading |

`ozton-math` and `ozton-control` have no VEX dependency. Standalone
`ozton-localization` consumes the hardware-independent observations from
`ozton_math::localization::observation`. Hosts can supply timestamps with
`Tracking::sample_at` or construct their own samples. `ozton-motion` with
`default-features = false` exposes
the differential controller, profiled pursuit, RAMSETE, and drive mixers.
The default `vex` feature enables async motion execution with VEX timers and
competition state. The `ozton` facade enables the VEX adapters for firmware.

Licensed under [MIT](LICENSE-MIT.md) or [Apache 2.0](LICENSE-APACHE.md).

## Firmware logs

Enable logging with `nix run .#upload -- --logging` from Override. The USB
terminal shows timestamped errors, warnings, and info with a source and message.
Insert a writable SD card before startup to append all five levels to
`OZTON.LOG`. The writer flushes about once a second. `SD_GAP` reports buffer
overflow or bytes whose write could not be confirmed.

View the log with `cargo run -p ozton-tools -- view-log /path/to/OZTON.LOG`.
Append `--kind MCL` or `--kind MOTION_STEP` to filter.

Schema 2 is UTF-8 with one `DBG,KIND,key=value,...,level=LEVEL` record per line.
`SESSION` starts each run. Split on commas and then on the first equals sign in
each field before percent-decoding text values. Numeric fields include units
where applicable. `NaN` marks an unavailable number. A schema 2 record without
a final level field may be a partial write. `t_ms` is program time in
milliseconds. Localization sample rows share `seq` and `t_ms`. Periodic robot
samples have their own sequence.

- `MOTION_STEP` and `PATH_FOLLOWER` show pose, progress, tracking error and wheel targets.
- `LOCALIZATION_POSE` shows odometry, prediction, published estimate and correction candidate.
- `MCL` shows particle pose, spread, correction gain and failure status.
- `DSR` shows reset decisions, observed axes and corrected pose.
- `LOCALIZATION_RANGE`, `DISTANCE_PACKET`, `TRACKING_INPUT` and `TRACKING_WHEEL`
  show sensor inputs and range rejection reasons.
- `MOTOR`, `DRIVE` and `IMU` show actuator commands and measured feedback.
