# Write autonomous routes

## Import a PATH.JERRYIO route

1. In [PATH.JERRYIO](https://path.jerryio.com/), use **LemLib v0.5** export
   and the **VEX Gaming Positioning System** coordinate setting.
2. Save one path per file and embed it with `include_bytes!`.
3. Parse the export with `jerryio::parse` and profile it with `Path::new` and
   the chassis limits before entering competition.
4. Register an async route with `autonomous_routes!` and install the route set
   with `Robot::with_autonomous::<Route>(display)`. A registered function
   receives `&mut CompetitionBot` and returns a `Result` using the error type
   from its `CompetitionControls` implementation.
5. While disabled, select the route on the Brain. Place the robot at the surveyed
   starting pose, facing along the path for forward travel.

The importer converts inches to metres and uses the exported speed as a cap
relative to the chassis limit. It strips the exporter's lookahead extension.
The file supplies positions and speed caps. Specify the initial heading in Rust.

```rust,ignore
{{#include ../packages/ozton/examples/programmed.rs:prepare_path}}
```

To obtain Rust waypoints for a custom path, run:

```sh
cargo run -p ozton-tools -- import-path route.txt
```

Use `jerryio::parse_chain` to combine exports in travel order before profiling
them as one continuous path. The junctions must meet within 2 mm and have
matching travel tangents. Use separate commands to change direction or stop
for a mechanism action.

## Compose motions and mechanism actions

Create one `Runner` per route and call `confirm_start` with the surveyed pose.
The programmed example retains the profiled path in its controls and clones the
`Rc` handle before the runner mutably borrows the robot.

```rust,ignore
{{#include ../packages/ozton/examples/programmed.rs:route}}
```

Use `Command::FollowMarked` with a slice of `(distance_m, action)` markers for
multiple actions. Markers run once in distance order, including all markers
crossed in one tick. On a chained path, measure from the start of the full path.
Set `reverse: true` for reverse travel and start with the robot facing against
the travel direction.

Set retained mechanism goals through `runner.robot()` or from a marker.
The runner calls `AutonomousControls::update_mechanisms` during movement
and waits. Use `wait_until(predicate, stable, timeout)` for a measured condition
such as a lift staying within tolerance for a settling interval. The predicate
can capture thresholds, reads robot state, and should complete quickly.

## Try alignment and continue

Set `Alignment::sensor` to the index of the range sensor in the feedback
array. Choose a clear view of the intended surface, a positive approach speed
in metres per second, and `direction` of `1.0` for a front-facing sensor or
`-1.0` for a rear-facing one. The configured chassis checks the range sample
for age, confidence, operating range, and occlusion. Field-map association is
used for localization.

A clean optional alignment timeout brakes the chassis and returns `false`.
The original route deadline and retained mechanism goals carry through. Field
disable, route deadline, tracking fault, control gap, or output failure stops
the route. Dropping the runner stops the chassis and mechanisms.

## Short commands, scoring and bounded fallback

`Runner` exposes `drive_to_point`, `drive_distance`, `turn_to`, and `wait`.
Positions and distances are metres. Headings are CCW radians from forward.
Absolute point, path, and DSR commands check pose trust every tick.
`AutonomousControls::pose_trusted` and `AutonRobot::pose_trusted` default to
false. Supply evidence of trust from the tracker.

`Command::DriveToPoint { target: [x, y], reverse, maximum_speed }` closes the
loop on field position, including lateral displacement. Use a subsequent
`Command::Turn` when a final heading is required. `Runner::try_distance_reset`
returns false on a clean DSR timeout and cancels the pending request before
allowing a fallback command.

Use `Command::ApproachContact` for a bounded approach with an
application-supplied contact predicate and settling time. The predicate can
inspect the robot, current feedback, and elapsed command time. After settling,
register a pose only if the contact geometry establishes that pose.

Implement `ScoringControls<CompetitionBot>` on the installed control state
with your own `Target`. `Runner::perform` waits for the measured-arrival
predicate and settling dwell, then runs a timed action callback. Define
scoring poses, physical units, gates, and actuator mappings in the application.
For a standalone robot adapter, implement `MechanismRobot` directly.

`try_run` returns false for a timed-out, stalled, lost-path, or untrusted-pose
optional stage. Handle that result explicitly and reuse the same runner for
the fallback so the original deadline still applies. `run` reports failure
and stops. Classify transient device errors with `recoverable` and implement
`pause` to neutralize outputs while preserving goals. Retries last at most
300 ms per failing subsystem. Sustained commanded motion without 8 mm translation
or 0.025 rad yaw progress for 800 ms stops the stage. Disable and expiry are
terminal.

`can_skip_failure` authorizes fallback after a device failure and a full stop.
It defaults to false. A skipped failure must clear retained unsafe goals.

Apply lift-height speed and turn limits and acceleration limits in the drive
adapter so they apply to every motion.

## Register routes with the stock differential adapter

With `motion::auton::Robot<M>` and `compete_routes`, build routes with
`Route::jerryio`, `Route::jerryio_chain`, or `Route::new` for a routine with
no embedded path. Each takes a menu label, a surveyed start pose, and an
async function receiving `Auton<'_, M>`. The stock runtime prepares embedded
paths at startup and registers the surveyed pose before the function runs.

Use `auton.follow(timeout)` for the registered path or `follow_with` to
supply a follower, reverse flag, and markers. `auton.range("front")?` resolves
a named sensor for `auton.try_align(sensor, target, timeout)`. Implement
`MechanismActions` on `M` to use `auton.perform(target, action_for, timeout)`.
