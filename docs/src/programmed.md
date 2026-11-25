# Programmed autonomous

An autonomous route is an async method on the application robot. Register it using `route!("Match", Self::route_match)` in `OztonCompete::ROUTES`.

Use `Basic` for distance, heading and point commands, `Boomerang` for continuous pose acquisition, and a typed trajectory follower for prepared paths. Import `ozton::motion::capabilities::*` to use the supported commands. Register the surveyed starting pose through the drivetrain localization before executing motion.

```rust,ignore
async fn route_match(&mut self) -> ControlResult {
    self.drivetrain.localization_mut().set_pose(start_pose);
    MOTION.drive_distance(&mut self.drivetrain, qty!(24 in)).await?;
    MOTION.turn_to_heading(&mut self.drivetrain, qty!(90 deg)).await?;
    MOTION.move_to_pose(&mut self.drivetrain, target_pose).await?;
    Ok(())
}
```

On the build host, use a native `Waypoint` slice and physical limits to prepare trajectory samples. Load the embedded samples with `Path::from_precomputed` on the Brain. Call `PurePursuit::follow_path` (through the `FollowPath` trait) for geometric pursuit, or select another typed trajectory follower.

Coordinate mechanisms through their own handles and scoped `TaskSlot` work. Do not hold mutable device borrows across unrelated awaits. Propagate motion faults and timeouts rather than assuming that an unsuccessful move reached its target.
