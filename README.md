# Ozton

Rust drivetrain, tracking, and competition control for VEX V5, built with vexide.

See [AGENTS.md](AGENTS.md) for development, deployment, and route documentation.
The [programmed example](packages/ozton/examples/programmed.rs) shows a complete robot.

Use `Pose::new(position, heading)` for a full destination and pass it to
`drivetrain.drive_to_point(target_pose)`. Positions use `DVec2` in right/forward
meters. Use `geometry::point(x, y)` to convert two unit-bearing lengths. Commands
that only need a heading, such as `turn_to`, take an `Angle`.

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
