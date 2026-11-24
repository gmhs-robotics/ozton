# Motion commands

Choose the simplest motion that solves the task. Drive straight when you need a distance. Turn when you need a heading. Use a pose command when the final location and orientation both matter.

## Motion controller

Seeking executes distances, turns, point and pose goals, arcs and prepared paths using calibrated physical wheel velocities, feedback and field clearance. Profiled paths support both RAMSETE and Pure Pursuit.

## A first closed-loop move

Once the localized drive has a registered starting pose, call the following from your autonomous callback:

```rust
use ozton::{qty, position_xyh, motion::{Seeking, SettlingConfig}};
let motion = Seeking::new().with_settling(
    SettlingConfig::new()
        .with_position_tolerance(qty!(0.75 in))
        .with_heading_tolerance(qty!(2 deg)),
);

motion.drive_distance(&mut drivetrain, qty!(12 in)).await?;
motion.turn_to_heading(&mut drivetrain, qty!(90 deg)).await?;
motion.move_to_point(&mut drivetrain, ozton::position_xy!(8 in, 12 in)).await?;
motion.move_to_pose(&mut drivetrain, position_xyh!(0 in, 0 in, 0 deg)).await?;
```

Both point and pose targets use the same controller configuration. `move_to_point` leaves the final heading free; `move_to_pose` requires a final heading and can plan a continuous trajectory. The robot must have room for these moves and a correct starting pose. The destination heading is absolute. A distance may be negative for reverse travel; `with_travel(TravelDirection::Backward)` selects a backward point or pose approach. drive_arc accepts a positive radius and signed sweep.

## Configure once

Declare a reusable controller with a const builder. Configure the controller before issuing a command; command futures only execute the selected settings. For a one-off change, use a modified copy of your constant. Every modifier returns the configured template, so no setup function or global tuning struct is needed.

```rust,ignore
use ozton::{qty, motion::{Seeking, MotionConfig, MotionLimits, DriveTuning}};
use ozton::math::control::Pid;

const MOTION: MotionConfig = MotionConfig::new()
    .with_tuning(
        DriveTuning::new()
            .with_linear_controller(Pid::new(4.25, 0.0, 0.50, None))
            .with_angular_controller(Pid::new(3.0, 0.0, 0.22, None)),
    )
    .with_limits(MotionLimits::new().with_wheel_speed(qty!(1.2 m/s)));
const SEEKING: Seeking = Seeking::new().with_config(MOTION);
```

A command finishes when the applicable error and velocity tolerances remain satisfied for the required settle duration. Seeking returns an error on timeout, invalid feedback or insufficient clearance. Tune physical velocity gains and braking rates on the actual robot.

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
crossing. For a yaw excursion theta, compute the maximum projection of each
rotated chassis corner and each signed travel interval onto the four
field axes. The support of a vector r along unit axis n is |r| if the
aligned direction is within the heading interval; otherwise it is
(r dot n) cos(theta) + |r cross n| sin(theta). Sum independent travel
supports and the maximum corner support. This conservatively bounds the
sweep without inflating the chassis beyond its actual turning disk.
Test the resulting AABB against the perimeter and hard obstacles. Loose landmarks are not hard
obstacles.

The braking model is conditional on its physical assumptions: the reaction
interval, minimum forward/lateral/angular braking rates, estimation error
and localization margin must cover the actual robot. Covariance is a model
uncertainty, not a deterministic guarantee against slip or bad sensors.
The default latency and lateral braking values are initial estimates.
Characterize them with measured stopping distances before increasing speed.
Configure the latency, preview, lateral braking and uncertainty in ClearanceConfig.


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
trajectory acceleration/deceleration using ClearanceConfig::with_forward_braking and with_yaw_braking. Never
infer guaranteed physical stopping performance from a commanded profile limit.

The command slew limiter records the wheel targets actually sent after wall
governance. Releasing a safety limit therefore resumes from the constrained
speed rather than the unconstrained internal target.


## Optional breakaway compensation

MotionLimits::with_minimum_linear_speed and with_minimum_angular_speed configure
measured breakaway rates; zero (default) disables them. The controller only
raises a nonzero error-reducing command when its measured axis is nearly at
rest and outside the arrival tolerance. Compensation is capped by the PID and
route limits and by the stopping speed obtained from d = v*T + v^2/(2*a),
where d is the remaining error outside the arrival band. Slew, curvature
desaturation and wall governance still apply afterward. Inside the arrival
band, during braking, or when the safety governor requires it, the output can
be below the configured minimum or zero. Profiled path tracking is unchanged
until terminal feedback takes over. Characterize linear and rotational
breakaway rates on the actual chassis rather than assuming universal values.

Path construction reuses segment lengths, the Hermite planner reuses its waypoint
buffer and skips clearance testing candidates slower than an already valid best
route. Path progress keeps a monotonic segment cursor instead of searching and
interpolating full pose/time profiles for every closest-point projection. Terminal
feedback skips the moving-reference calculation when it will not be used.

## Composable motion configuration

MotionConfig is a const-friendly collection of typed sub-configurations:
DriveTuning, MotionLimits, RateFilters, SettlingConfig, MotionTiming,
ClearanceConfig, PathConfig and ApproachConfig. Each supports defaults and
const builder methods; configure only the differences from defaults. Pass a
complete configuration with Seeking::with_config, or change one group with
with_limits, with_safety, with_settling and the other group setters. Copy a configuration group and adjust only the settings needed for a per-route override. The public fields are now grouped (for example,
config.limits.wheel_speed and config.safety.forward_braking).
