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
