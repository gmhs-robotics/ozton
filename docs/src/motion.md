# Motion commands

Choose the simplest motion that solves the task. Drive straight when you need a distance. Turn when you need a heading. Use a pose command when the final location and orientation both matter.

## Which controller should I use?

- Basic: distance driving and heading or point turns, using normalized motor output and the tracking required by the command.
- Seeking: move toward a point, with a choice of forward or reverse approach.
- PurePursuit: follow waypoints with normalized motor output and an endpoint controller.
- PoseSeeking: calibrated physical wheel velocities, field pose feedback, wall clearance and continuous pose planning for differential drives. Supports distance, turn, arc, point, pose and prepared profiled path commands.

The first three expose Evian-style motion modifiers. Their PID output is a motor fraction. PoseSeeking's outer loops instead produce physical linear and angular velocity. A gain tuned for one is not automatically suitable for another.

## A first closed-loop move

Once the localized drive has a registered starting pose, call the following from your autonomous callback:

```rust
use ozton::{length, angle, position_xyh, motion::PoseSeeking};
let motion = PoseSeeking::default()
    .with_position_tolerance(length!(0.75 in))
    .with_heading_tolerance(angle!(2 deg));

motion.drive_distance(&mut drivetrain, length!(12 in)).await?;
motion.turn_to_heading(&mut drivetrain, angle!(90 deg)).await?;
motion.move_to_pose(&mut drivetrain, position_xyh!(0 in, 0 in, 0 deg)).await?;
```

The robot must have room for these moves and a correct starting pose. The destination heading is absolute. A distance may be negative for reverse travel; reverse() selects a backward point or pose approach. drive_arc accepts a positive radius and signed sweep.

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

Seeking, PurePursuit and PoseSeeking have the same construction style. Supply the actual track width to PurePursuit with `with_track_width`. PoseSeeking offers physical speed, acceleration, PID, settling and clearance modifiers directly; MotionConfig remains available for the generic Runner. PID controllers also support const gain, integration range and output-limit modifiers. Library defaults are starting points, not calibration results.

A command finishes when its applicable error and velocity tolerances remain satisfied for the required settle duration. PoseSeeking returns an error on timeout, loss of feedback or insufficient clearance. Generic motion commands return MotionOutcome::Settled or MotionOutcome::TimedOut; check the outcome rather than assuming an Ok means the target was reached.

## Following a prepared path

Create a Path from waypoints and physical limits in ozton::math::path. Use PoseSeeking::follow_path with a profiled Follower (RAMSETE or PurePursuit). The final stage settles against pose feedback. A path generated from a map is not a guarantee against obstacles: inspect the real field and keep disable available.
