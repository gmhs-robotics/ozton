# Write autonomous routes

## Import a PATH.JERRYIO route

1. In [PATH.JERRYIO](https://path.jerryio.com/), use **LemLib v0.5** export
   and the **VEX Gaming Positioning System** coordinate setting.
2. Save one path per file and embed it with `include_bytes!`.
3. Register a name, a starting pose and an async Rust function with
   `Route::jerryio`. Ozton parses and profiles each route before competition.
4. Select the route on the Brain before enabling autonomous. Put the robot at
   the declared starting pose and facing along the path for forward travel.

The importer converts inches to meters and uses the exported speed as a cap
relative to the chassis limit. Ozton strips the exporter’s lookahead extension.
The file supplies positions and speed caps. Specify the initial heading in Rust.
To obtain Rust waypoints for a custom path, run:

```sh
cargo run -p ozton-tools -- import-path route.txt
```

## Compose motions and mechanism actions

Inside a route function, `auton.follow(timeout)` follows the registered path
with Pure Pursuit. `follow_with` selects the controller, forward or reverse
travel, and a slice of `(distance_m, action)` markers. Markers run once in
distance order, including all markers crossed in a single tick. Reverse paths
start with the robot facing against the travel direction.

Set retained mechanism goals through `auton.mechanisms()` or a marker. Ozton
updates the mechanisms while the robot is moving and waiting. Use
`wait_until(predicate, timeout)` for a measured condition such as a lift reaching
its setpoint. The predicate reads robot state and should complete quickly.

`Route::jerryio_chain` takes multiple exports in travel order and profiles them
as one continuous path. The junctions must meet within 2 mm and have matching
travel tangents. Markers use distance along the combined path. Use separate
commands when changing direction or stopping for a mechanism.

`Route::new` registers a routine without an embedded path. Use
`auton.run(command, timeout)` for a signed distance, turn, contact, DSR, or wait.

## Try alignment and continue

Use `auton.range("front")?` to resolve a configured range device. An alignment
requires a clear view of the intended surface. Ozton checks the distance sample
for age, confidence, operating range and occlusion. Field-map association is
reserved for localization.

```rust,ignore
let front = auton.range("front")?;
let aligned = auton
    .try_align(front, AlignTarget::front(0.12, 0.0), Duration::from_secs(1))
    .await?;

if aligned {
    auton.mechanisms().start_scoring();
}
```

A clean optional alignment timeout brakes the chassis and returns `false`.
Mechanism goals and the original route deadline carry through. A controller
disable, route deadline, tracking fault, control gap, or output failure stops
the route. Dropping an in-flight command stops chassis and mechanisms.

`Command::DriveToPoint { target: [x, y], reverse, maximum_speed }` closes the loop
on full field position, including measured lateral displacement. Coordinates use
the native Ozton field frame in metres. It stops at the point without imposing a
final heading; use `Command::Turn` afterward when a final heading is required.
`Runner::try_distance_reset(timeout)` returns false on a clean DSR timeout and
cancels the pending request before allowing a fallback command. The single runner
continues servicing retained mechanism targets under its original route deadline.

## Short commands, scoring and bounded fallback

`Runner` and `Auton` expose `drive_to_point`, `drive_distance`, `turn_to` and `wait`.
Positions and distances are metres; headings are CCW radians. `Runner::confirm_start`
attests a surveyed starting pose. Absolute point/path/DSR commands refuse untrusted
pose every tick. `AutonRobot::pose_trusted` defaults to false, so custom adapters must
provide evidence explicitly. Registering a `Route` supplies its declared surveyed start.

Implement `MechanismRobot` for an application-owned target type. `Runner::perform`
waits for the application's measured-arrival predicate and settling dwell, then
runs a timed action callback. The library assumes no lift, pivot, ejection hardware
or approach/retreat geometry. Define scoring poses, physical units, gates and actuator
mappings in the robot application, and stop all outputs in `AutonRobot::stop`.

`try_run` returns false for a timed-out, stalled, lost-path or untrusted-pose optional
stage. Choose a fallback explicitly. Its original route deadline is unchanged.
`run` still reports failure and stops; ignoring an error never silently continues a
route. Custom adapters can classify transient device errors with `recoverable` and
provide `pause` to neutralize outputs while preserving goals. Retries last at most
300 ms per failing subsystem. Sustained commanded motion without 8 mm translation or
0.025 rad yaw progress for 800 ms stops the stage. Disable and expiry remain terminal.

The robot's stability policy belongs in its drive adapter, so every motion receives
current lift-height speed/turn limits and acceleration limits. It must not depend on
route authors remembering to set a cap before each command.

For the stock `Robot<M>`, implement `MechanismActions` on `M` with its own `Target`;
then a registered route can call `auton.perform(target, action_duration, timeout)`.
Custom robot adapters implement `MechanismRobot`. A robot may wrap this as `score`
in its own source tree, with all mechanism-specific targets and behavior there.

`AutonRobot::can_skip_failure` separately authorizes full-stop fallback after a device
failure. It defaults to false. This differs from `recoverable`, which permits a bounded
retry of the same operation. A skipped failure must clear retained unsafe goals.
