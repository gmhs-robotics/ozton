# Motion commands

Choose the simplest motion that solves the task. Drive straight when you need a distance. Turn when you need a heading. Use a pose command when the final location and orientation both matter.

## Motion controller

Controllers implement independent motion capability traits, borrowing the drivetrain only for a command. `Basic` supports distance, point, heading, swing, arc, straight pose commands. `Boomerang` supports continuous pose acquisition. `PurePursuit`, `Ramsete` and `LtvLqr` implement `FollowPath`. All share calibrated physical velocity feedback, and cancellation handling.

## A first closed-loop move

MotionExecution requires calibrated drive encoders and localization. A normal Differential model supports manual drive; attach the physical footprint with `.with_footprint(footprint)` for surveyed pose registration, or use `.with_unbounded_motion()` when no footprint is needed. Neither option enables obstacle checking. Once the localized drive has a registered starting pose, call the following from your autonomous callback:

```rust
use ozton::{qty, position_xyh, motion::{Basic, Boomerang, SettlingConfig, capabilities::*}};
let motion = Basic::new().with_settling(
    SettlingConfig::new()
        .with_position_tolerance(qty!(0.75 in))
        .with_heading_tolerance(qty!(2 deg)),
);

motion.drive_distance(&mut drivetrain, qty!(12 in)).await?;
motion.turn_to_heading(&mut drivetrain, qty!(90 deg)).await?;
motion.move_to_point(&mut drivetrain, ozton::position_xy!(8 in, 12 in)).await?;
Boomerang::new().with_config(motion.config())
    .move_to_pose(&mut drivetrain, position_xyh!(0 in, 0 in, 0 deg)).await?;
```

Both point and pose targets use the same controller configuration. `move_to_point` leaves the final heading free; `Boomerang::move_to_pose` uses allocation-free continuous Boomerang feedback toward a final heading. `Basic::move_to_pose` uses straight legs and stationary turns. The robot must have room for these moves and a correct starting pose. The destination heading is absolute. A distance may be negative for reverse travel; `with_travel(TravelDirection::Backward)` selects a backward point or pose approach. drive_arc accepts a positive radius and signed sweep.

## Typed driver assistance

Supply the orientation source through Sources::with_orientation.
HeadingHoldAssist requires ReadHeading; AntiTipAssist
requires ReadTilt, which supplies pitch and roll in that order. A source without
the required trait prevents constructing AssistedDrivetrain. Custom sensors may
implement either capability without depending on VEX.

```rust,ignore
let mut assists = DriveAssists::new((
    HeadingHoldAssist::new(1.5, 0.2),
    AntiTipAssist::new(TiltAxis::Pitch, anti_tip_controller),
    MotorBalanceAssist,
));
let mut drive = AssistedDrivetrain::new(&mut drivetrain, &mut assists);
```

DSR requires RangeResetDrive, which the built-in drivetrain implements only for a nonempty range bank. Only installed assists read the corresponding capability. A temporarily unavailable
sensor observation resets that controller rather than pretending a zero angle.

## Configure once

Declare a reusable controller with a const builder. Configure the controller before issuing a command; command futures only execute the selected settings. For a one-off change, use a modified copy of your constant. Every modifier returns the configured template, so no setup function or global tuning struct is needed.

```rust,ignore
use ozton::{qty, motion::{Basic, MotionConfig, MotionLimits, DriveTuning}};
use ozton::math::control::Pid;

const MOTION: MotionConfig = MotionConfig::new()
    .with_tuning(
        DriveTuning::new()
            .with_linear_controller(Pid::new(4.25, 0.0, 0.50, None))
            .with_angular_controller(Pid::new(3.0, 0.0, 0.22, None)),
    )
    .with_limits(MotionLimits::new().with_wheel_speed(qty!(1.2 m/s)));
const BASIC: Basic = Basic::new().with_config(MOTION);
```

A command finishes when the applicable error and velocity tolerances remain satisfied for the required settle duration. Commands return errors on invalid feedback or cancellation; progress and stage timeouts are diagnostic thresholds in the current executor. Tune physical velocity gains and braking rates on the actual robot.

## Following a prepared path

Build a `Path` on the host with the robot's physical limits, then embed the samples and `limits.as_array()`. Use `motion::prepare_path` to validate the generated limits against the live calibration and the selected controller’s `.run(&mut drive, &path)` to execute it. Use `.follow(&path).with_actions(actions).run(&mut drive)` for nonblocking mechanism callbacks. The final stage settles against pose feedback.

```rust,ignore
use ozton::motion::{PurePursuit, prepare_path};
let path = prepare_path(&drivetrain, MOTION, SAMPLES, GENERATED_LIMITS)?;
PurePursuit::new().with_config(MOTION)
    .run(&mut drivetrain, &path).await?;
```

Select a controller by constructing its type, for example `Ramsete::new().with_config(MOTION).with_gains(2.0, 0.7)`. `LtvLqr` computes finite-horizon discrete Riccati gains along the curved, timed trajectory before motion begins. Reusable mathematical laws live in `ozton::math::following`; motion wrappers select references, apply physical limits, and execute the commands. A path generated from a map is not a guarantee against obstacles: inspect the real field and keep disable available.


## Chaining motions

Use the same runnable motion descriptions for a single command or a chain:

```rust,ignore
use ozton::motion::{MotionChain, PurePursuit, Ramsete, Boomerang};
PurePursuit::new().with_config(MOTION).follow(&path_a).run(&mut drivetrain).await?;

MotionChain::new()
    .then(PurePursuit::new().with_config(MOTION).follow(&path_a))
    .then(Ramsete::new().with_config(MOTION).follow(&path_b))
    .then_stop(Boomerang::new().with_config(MOTION).to_pose(goal))
    .run(&mut drivetrain).await?;
```

Compatible tangent-connected paths share one velocity profile, including
authored speed caps and the most restrictive physical limits. Identical
controller configurations keep the same follower state. Different laws
initialize at measured progress and retain the shared wheel limiter.
Direction changes with matching body heading brake to zero without requiring
a dwell when within arrival tolerances. Incompatible geometry, direct point,
heading and pose primitives, explicit stops and the final command settle
normally. Use `PathChain` only to generate geometry, not to execute motions.

`RunMotion` is the common trait for a bound motion and a chain. Path followers
also support `controller.run(&mut drivetrain, &path)` as shorthand.
`MotionSegment::named` labels checkpoint telemetry; `with_actions` attaches
nonblocking path actions.

## Optional controller features

Classical Pure Pursuit uses a fixed lookahead radius and the measured pose by
default. A lookahead radius is intrinsic to Pure Pursuit; adaptive lookahead,
pose prediction, and behind-target PID alignment are independent opt-in
extensions. Configure them directly on the controller after applying shared
robot settings:

```rust,ignore
use std::time::Duration;
use ozton::{qty, motion::PurePursuit};
let classical = PurePursuit::new().with_config(MOTION).with_lookahead(qty!(0.23 m));
let adaptive = classical.with_adaptive_lookahead(Duration::from_millis(320));
let predictive = adaptive
    .with_prediction_latency(Duration::from_millis(125), Duration::from_millis(160))
    .with_heading_recovery(true);
```

The corresponding PathConfig builders are available for shared configuration
and host simulation. Adaptive lookahead is expressed as a time horizon rather
than a multiplier of prediction latency. RAMSETE and LTV-LQR use their own
gain and weight builders and do not use pursuit-specific extensions. Endpoint
settling remains shared; opt into polar feedback with `with_polar_terminal`.
Boomerang's lead fraction is configured with `with_lead`.

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

Pure Pursuit uses measured geometric progress. RAMSETE and LTV-LQR track the time-parameterized trajectory. Mechanism progress callbacks still use measured geometric progress.
The terminal controller uses a rotate-drive-rotate PID sequence, or optional polar-coordinate pose feedback, for convergence after tracking. Use
PathConfig::with_maximum_cross_track_error to set a diagnostic warning threshold. It does not abort the path.

## Braking and motion policy

Motion controllers execute the requested targets without implicit obstacle,
wall-clearance, or swept-footprint checks. Field geometry is an observation
model for localization, not a motion-stop policy. Do not introduce automatic
collision preflight or clearance governors into ordinary motion commands.
Explicit competition exclusion boundaries belong in a separately configured
policy; none are enabled by default. Runtime feedback remains constrained by
the configured wheel speeds, acceleration and lateral acceleration limits.

Interrupted motions, terminal capture and feedback faults request explicit
motor electrical braking rather than normal resting coast. A zero-RPM
velocity target is not equivalent to this braking command. Characterize the
active response, actuator latency and traction on the physical robot.

## Authored routes

No command searches for another geometric route. Direct feedback follows the selected target; prepared trajectories preserve the supplied curve and speed profile. Choose explicit straight poses when stationary turns are required, and build a single continuous PathChain or authored curve on the host when uninterrupted motion is required.

The command slew limiter records the applied wheel targets so subsequent
commands begin from the previous constrained output.

## Optional breakaway compensation

MotionLimits::with_minimum_linear_speed and with_minimum_angular_speed configure
measured breakaway rates; zero (default) disables them. The controller only
raises a nonzero error-reducing command when its measured axis is nearly at
rest and outside the arrival tolerance. Compensation is capped by the PID and
route limits and by the stopping speed obtained from d = v*T + v^2/(2*a),
where d is the remaining error outside the arrival band. Slew and curvature desaturation still apply afterward. Inside the arrival
band, or during braking, the output can
be below the configured minimum or zero. Profiled path tracking is unchanged
until terminal feedback takes over. Characterize linear and rotational
breakaway rates on the actual chassis rather than assuming universal values.

Host path construction reuses segment lengths. Path progress keeps a monotonic segment cursor instead of searching and
interpolating full pose/time profiles for every closest-point projection. Terminal
feedback skips the moving-reference calculation when it will not be used.

## Composable motion configuration

MotionConfig is a const-friendly collection of typed sub-configurations:
DriveTuning, MotionLimits, RateFilters, SettlingConfig, MotionTiming,
PathConfig and ApproachConfig. Each supports defaults and
const builder methods; configure only the differences from defaults. Pass a
complete configuration with the controller's `with_config`, or change limits and settling with the controller's builder methods. Copy a configuration group and adjust only the settings needed for a per-route override. The public fields are now grouped (for example,
config.limits.wheel_speed and config.limits.forward_braking).

Swing turns require both `MotionExecution` and `WheelActuation`. Ordinary
wheel actuation is independent of localization; the swing controller composes
these capabilities instead of requiring a particular drivetrain or motor type.
