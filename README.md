# Ozton

Rust drivetrain, tracking, and competition control for VEX V5, built with vexide.

See [AGENTS.md](AGENTS.md) for development, deployment, and route documentation.
The [programmed example](packages/ozton/examples/programmed.rs) shows a complete robot.

Use `Pose::new(position, heading)` for a full destination and pass it to
`drivetrain.drive_to_point(target_pose)`. Positions use `DVec2` in right/forward
meters. Use `geometry::point(x, y)` to convert two unit-bearing lengths. Commands
that only need a heading, such as `turn_to`, take an `Angle`.

Licensed under [MIT](LICENSE-MIT.md) or [Apache 2.0](LICENSE-APACHE.md).
