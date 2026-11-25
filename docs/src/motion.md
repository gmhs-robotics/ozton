# Motion commands

Choose the simplest motion that solves the task. Drive straight when you need a distance. Turn when you need a heading. Use a pose command when the final location and orientation both matter.

## Motion controller

Seeking executes distances, turns, point and pose goals, arcs and prepared paths using calibrated physical wheel velocities, feedback and field clearance. Profiled paths support both RAMSETE and Pure Pursuit.

## A first closed-loop move

PoseDrive requires calibrated drive encoders and an explicit geometry policy. A normal Differential model supports manual drive; call `.with_footprint(footprint)` to obtain a field-bounded model, or `.with_unbounded_motion()` to deliberately operate without clearance checking. Bounded motion still requires a valid field map. Once the localized drive has a registered starting pose, call the following from your autonomous callback:

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

## Typed driver assistance

Select a source by its tuple index, preferably a named constant declared next to
the robot's hardware layout. HeadingHoldAssist requires ReadHeading; AntiTipAssist
requires ReadTilt, which supplies pitch and roll in that order. A source without
the required trait prevents constructing AssistedDrivetrain. Custom sensors may
implement either capability without depending on VEX.

```rust,ignore
let mut assists = DriveAssists::new((
    HeadingHoldAssist::<IMU_SOURCE>::new(1.5, 0.2),
    AntiTipAssist::<IMU_SOURCE>::new(TiltAxis::Pitch, anti_tip_controller),
    MotorBalanceAssist,
));
let mut drive = AssistedDrivetrain::new(&mut drivetrain, &mut assists);
```

Only installed assists read the corresponding capability. A temporarily unavailable
sensor observation resets that controller rather than pretending a zero angle.

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


## Wheel-constrained trajectories

Path construction applies forward and backward reachability in squared speed. For each
segment, curvature and squared speed are interpolated in arc length. The differential
wheel acceleration constraints are linear in the two endpoint squared speeds, so
the backward pass projects a small set of inequalities and the forward pass chooses
the greatest attainable successor. This limits both wheels across the whole segment,
including curvature changes. Use MotionLimits::with_wheel_acceleration (or
path::Limits::with_wheel_acceleration) to supply a characterized wheel acceleration
limit. When unspecified, path construction uses the nominal linear acceleration.
The live controller also constrains feedback wheel slew when explicitly configured.

The moving RAMSETE reference is limited by arc-length lead over geometric progress.
The same terminal pose controller handles convergence for both followers. Use
PathConfig::with_maximum_cross_track_error to set the allowed tracking deviation.

## Predictive stopping and wall approach

The hard boundary is checked against the entire chassis and its reachable
braking sweep. The reaction interval combines maximum control gap and brake
latency. The command governor adds preview and scales both wheel requests
together. It bounds each signed rate using measured and requested velocities,
including the preceding applied command while it may still be in flight and
estimated rate uncertainty. With a characterized maximum physical
acceleration, integrate the acceleration ramp during the reaction interval;
otherwise conservatively allow an immediate transition to the larger rate.
After reaction, integrate braking using the minimum physical deceleration.

Signed yaw intervals determine the extreme projections of chassis corners
and independent forward/lateral travel. XY covariance from the active EKF
or MCL expands the projected bounds by the configured sigma multiplier.
Odometry-only or custom estimators without covariance use the configured
clearance margin, not invented covariance. Test the resulting AABB against
the perimeter and hard obstacles.

Interrupted motions, terminal capture and feedback faults request an active
zero-speed stop rather than the normal resting coast. The VEX adapter attempts
motor electrical braking if a zero-speed command fails. Calibrate the active
response separately from coast.

These bounds are conditional on the physical model. In particular, command
overshoot, slip, battery variation, sensor bias and underestimated covariance
can invalidate the assumed rate and error intervals. Do not treat a covariance
ellipse as a deterministic containment guarantee. Characterize forward/yaw
acceleration bounds, braking, latency and lateral stopping with physical tests.
Configure these independently of the nominal trajectory limits.

## Orientation-aware routing

Seeking first tries a continuous Hermite maneuver for a pose goal. If none is
feasible, LatticePlanner searches position and heading with bounded A-star,
using swept straight translations and actual shortest in-place rotations.
Exact connectors reach the requested pose without grid snapping. Adjacent
straight edges are merged. The discrete planner is bounded and can miss
feasible curved routes; the existing wall departure arc remains a recovery
option. Prefer plans avoiding movable objects, but never turn them into hard
runtime clearance constraints. The standalone visibility-graph AStar remains
available for position-only planning.

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
