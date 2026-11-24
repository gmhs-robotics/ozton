# Ozton

Rust drivetrain, tracking, and competition control for VEX V5, built with vexide.

See [AGENTS.md](AGENTS.md) for development, deployment, and route documentation.
The [programmed example](packages/ozton/examples/programmed.rs) shows a complete robot.

## Autonomous motion

Pass a mutable drivetrain to each motion. For a calibrated differential drive
with Ozton's live field localization, `PoseSeeking` plans clearance from walls,
settles with wheel and heading feedback, and stops on disable or timeout.

```rust
let seeking = PoseSeeking::default()
    .with_position_tolerance(Length::new::<inch>(0.75))
    .with_heading_tolerance(Angle::new::<degree>(2.0));

seeking.move_to_pose(&mut drivetrain, Pose::default()).await?;
seeking.reverse().move_to_point(&mut drivetrain, (0.45, 0.60)).await?;
```

Use `with_tuning(DriveTuning { linear, angular })` to supply both PIDs for a
chassis. The outer loops command metres per second and radians per second.
`with_wheel_speed` limits the wheels. `with_timeout` sets a per-stage timeout,
and `with_budget` limits the full departure, approach and final turn.

`Seeking` and `Basic` work with any `Arcade` drivetrain whose tracker supplies
the required feedback. Ozton provides `ozton::tracking::WheeledTracking` for
wheeled odometry. Point coordinates are right and forward in metres. Use
`geometry::point(x, y)` to convert two unit-bearing lengths.

```rust
let seeking = Seeking::default();
let outcome = seeking
    .move_to_point(&mut drivetrain, (0.61, 0.61))
    .reverse()
    .with_angular_kp(0.4)
    .without_timeout()
    .with_linear_voltage_limit(Motor::V5_MAX_VOLTAGE * 0.75)
    .await?;

let basic = Basic::default();
let outcome = basic.drive_distance(&mut drivetrain, Length::new::<inch>(24.0)).await?;
```

The generic motions return `MotionOutcome::Settled` or `MotionOutcome::TimedOut`.
`with_linear_output_limit(0.75)` sets the same throttle cap as the voltage form.
For a custom motion, write an async function that takes `&mut Drivetrain` and
requires only the tracking traits it uses.

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
