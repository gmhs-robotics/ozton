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
use ozton::{length, angle, position_xyh, motion::Seeking};
let motion = Seeking::new()
    .with_position_tolerance(length!(0.75 in))
    .with_heading_tolerance(angle!(2 deg));

motion.drive_distance(&mut drivetrain, length!(12 in)).await?;
motion.turn_to_heading(&mut drivetrain, angle!(90 deg)).await?;
motion.move_to_point(&mut drivetrain, ozton::position_xy!(8 in, 12 in)).await?;
motion.move_to_pose(&mut drivetrain, position_xyh!(0 in, 0 in, 0 deg)).await?;
```

Both point and pose targets use the same controller configuration. `move_to_point` leaves the final heading free; `move_to_pose` (or `move_to_point_at_heading`) requires a final heading and can plan a continuous trajectory. The robot must have room for these moves and a correct starting pose. The destination heading is absolute. A distance may be negative for reverse travel; reverse() selects a backward point or pose approach. drive_arc accepts a positive radius and signed sweep.

## Configure once

Declare a reusable controller with a const builder. Every modifier returns the configured template, so no setup function or global tuning struct is needed.

```rust,ignore
use ozton::{
    angle, length, motion::Basic,
    math::units::{velocity, angular_velocity},
};
use std::time::Duration;

const BASIC: Basic = Basic::new()
    .with_linear_gains(1.2, 0.0, 0.06)
    .with_angular_gains(1.6, 0.0, 0.08)
    .with_linear_error_tolerance(length!(0.75 in))
    .with_angular_error_tolerance(angle!(2 deg))
    .with_linear_velocity_tolerance(velocity!(0.04, meter_per_second))
    .with_angular_velocity_tolerance(angular_velocity!(0.12, radian_per_second))
    .with_timeout(Duration::from_secs(6));
```

Seeking and PurePursuit have the same construction style. Supply the actual track width to PurePursuit with `with_track_width`. Seeking offers physical speed, acceleration, PID, settling and clearance modifiers directly; MotionConfig remains available for the generic Runner. PID controllers also support const gain, integration range and output-limit modifiers. Library defaults are starting points, not calibration results.

A command finishes when its applicable error and velocity tolerances remain satisfied for the required settle duration. Seeking returns an error on timeout, loss of feedback or insufficient clearance. Basic, PurePursuit and normalized Seeking return MotionOutcome::Settled or MotionOutcome::TimedOut; check the outcome rather than assuming an Ok means the target was reached.

## Following a prepared path

Create a Path from waypoints and physical limits in ozton::math::path. Use Seeking::follow_path with a profiled Follower (RAMSETE or PurePursuit). The final stage settles against pose feedback. A path generated from a map is not a guarantee against obstacles: inspect the real field and keep disable available.
