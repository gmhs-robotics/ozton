# Motion commands

Choose the simplest motion that solves the task. Drive straight when you need a distance. Turn when you need a heading. Use a pose command when the final location and orientation both matter.

## Which controller should I use?

- Basic: distance driving and heading or point turns, using normalized motor output and the tracking required by the command.
- Seeking: move to a point, optionally specify the final heading, or execute a distance, turn, arc or prepared path. The default backend uses calibrated wheel velocities, feedback and field clearance.
- PurePursuit: follow waypoints with normalized motor output and an endpoint controller.

Basic and PurePursuit use normalized motor fractions. Seeking's default outer loops produce physical linear and angular velocities. For simple drivetrains without calibrated wheel-speed output, `Seeking::normalized()` opts into the normalized voltage controller for point and pose targets. Its gains use motor fractions instead; do not reuse physical velocity gains in that backend. It turns in place at the endpoint when a final heading is requested and does not perform physical trajectory planning or footprint checks.

## A first closed-loop move

Once the localized drive has a registered starting pose, call the following from your autonomous callback:

```rust
use ozton::{qty, position_xyh, motion::Seeking};
let motion = Seeking::new()
    .with_position_tolerance(qty!(0.75 in))
    .with_heading_tolerance(qty!(2 deg));

motion.drive_distance(&mut drivetrain, qty!(12 in)).await?;
motion.turn_to_heading(&mut drivetrain, qty!(90 deg)).await?;
motion.move_to_point(&mut drivetrain, ozton::position_xy!(8 in, 12 in)).await?;
motion.move_to_pose(&mut drivetrain, position_xyh!(0 in, 0 in, 0 deg)).await?;
```

Both point and pose targets use the same controller configuration. `move_to_point` leaves the final heading free; `move_to_pose` requires a final heading and can plan a continuous trajectory. The robot must have room for these moves and a correct starting pose. The destination heading is absolute. A distance may be negative for reverse travel; `with_travel(TravelDirection::Backward)` selects a backward point or pose approach. drive_arc accepts a positive radius and signed sweep.

## Configure once

Declare a reusable controller with a const builder. Configure the controller before issuing a command; command futures only execute the selected settings. For a one-off change, use a modified copy of your constant. Every modifier returns the configured template, so no setup function or global tuning struct is needed.

```rust,ignore
use ozton::{qty, motion::Basic};
use std::time::Duration;

const BASIC: Basic = Basic::new()
    .with_linear_gains(1.2, 0.0, 0.06)
    .with_angular_gains(1.6, 0.0, 0.08)
    .with_linear_error_tolerance(qty!(0.75 in))
    .with_angular_error_tolerance(qty!(2 deg))
    .with_linear_velocity_tolerance(qty!(0.04 m/s))
    .with_angular_velocity_tolerance(qty!(0.12 rad/s))
    .with_timeout(Duration::from_secs(6));
```

Seeking and PurePursuit have the same construction style. Physical settings require typed quantities, for example `Seeking::new().with_angular_acceleration(qty!(12 rad/s^2))`. Passing `12.0` alone will not compile. Supply the actual track width to PurePursuit with `with_track_width`. Seeking offers physical speed, acceleration, PID, settling and clearance modifiers directly; MotionConfig remains available for the generic Runner. PID controllers also support const gain, integration range and output-limit modifiers. Library defaults are starting points, not calibration results.

A command finishes when its applicable error and velocity tolerances remain satisfied for the required settle duration. Seeking returns an error on timeout, loss of feedback or insufficient clearance. Basic, PurePursuit and normalized Seeking return MotionOutcome::Settled or MotionOutcome::TimedOut; check the outcome rather than assuming an Ok means the target was reached.

## Following a prepared path

Create a Path from waypoints and physical limits in ozton::math::path. Use Seeking::follow_path with a profiled Follower (RAMSETE or PurePursuit). The final stage settles against pose feedback. A path generated from a map is not a guarantee against obstacles: inspect the real field and keep disable available.


## Predictive stopping and wall approach

The hard boundary is checked against the entire chassis and its reachable
braking sweep. For a signed rate v, a minimum measured braking deceleration
a and maximum reaction interval T, the signed stopping displacement is
d = v T + v |v| / (2 a). The reaction interval combines the maximum control
gap and configured actuator brake latency. The command governor adds a
configurable preview interval; its scalar bisection scales both wheels
together, preserving their differential curvature.

The velocity estimate's covariance defines an interval v ± k sigma for
each axis. Propagate the endpoints through the monotone stopping equation,
including zero; forward and lateral displacement bounds are combined
independently, so lateral motion cannot cancel a potential forward wall
crossing. For a yaw excursion theta, each corner and each translational
increment may rotate through a chord bounded by min(2, |theta|) times its
length. Inflate the swept AABB by that bound, then test the wall rectangle
and any hard obstacle bounds separately. Loose landmarks are not hard
obstacles.

The braking model is conditional on its physical assumptions: the reaction
interval, minimum forward/lateral/angular braking rates, estimation error
and localization margin must cover the actual robot. Covariance is a model
uncertainty, not a deterministic guarantee against slip or bad sensors.
The default latency and lateral braking values are initial estimates.
Characterize them with measured stopping distances before increasing speed.
Configure with MotionConfig::with_braking_model or Seeking::with_braking_model.


## Movable-obstacle routing

RectangularClearance::plan_soft_route uses a visibility graph over the corners
of obstacle boxes inflated by the chassis turning radius and clearance.
Each edge is clipped against inflated rectangles by the slab algorithm.
Dijkstra minimizes the total length inside soft-obstacle boxes first and the
travel length second, while hard obstacles and perimeter cannot be crossed.
This permits escape from a moved object that already overlaps the chassis
without treating that object as a hard boundary. The graph assumes the robot
can pivot at each waypoint. If a start is too close to a wall to pivot, the
regular clearance-aware departure planner is required.

The minimum braking deceleration can be configured independently of nominal
trajectory acceleration/deceleration using with_braking_deceleration. Never
infer guaranteed physical stopping performance from a commanded profile limit.

The command slew limiter records the wheel targets actually sent after wall
governance. Releasing a safety limit therefore resumes from the constrained
speed rather than the unconstrained internal target.
