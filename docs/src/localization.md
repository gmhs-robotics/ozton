# Tracking and localization

Tracking reads devices. Localization interprets the readings and estimates the robot's field pose. Keeping these separate means a different estimator does not require rewiring your robot.

## Assemble a tracked drivetrain

Create the motor groups once with shared_motors!, then use Differential::from_shared and DifferentialTracking::from_shared (or its calibrated constructors) so motor commands and encoders refer to the same devices. Give DifferentialTracking the actual wheel diameter, track width and gear ratios in motor order. An IMU and a lateral tracking wheel can improve the estimate.

Distance trackers already carry a default acceptance gate, zero bias and 25 mm measurement noise. Construct them directly with `DistanceTracker::at_mount(sensor, mount)`; add `.with_bias(...)`, `.with_sigma(...)` or `.with_gate(...)` on that tracker only when calibration requires it.

Register a flat tuple of sources with Tracking::new. This fragment assumes you have already constructed the devices, selected your field and calibrated the geometry:

```rust
use ozton::{
    drivetrain::Drivetrain,
    localization::{Estimator, differential::{LocalizationBuilder, DriveEncoderUse}},
    tracking::Tracking,
};

let tracking = Tracking::new((drive_encoders, lateral_wheel, imu, front_range)).await?;
let localization = LocalizationBuilder::new(field)
    .with_estimator(Estimator::Odometry)
    .with_imu_heading(true)
    .with_drive_encoders(DriveEncoderUse::ForwardAndHeading)
    .build(tracking.observation_layout())
    .expect("check tracking geometry");
let drivetrain = Drivetrain::new(model, tracking, localization);
```

The fragment is not a complete robot; field, model and sensor objects are your measured hardware. See the [programmed example](https://github.com/gmhs-robotics/ozton/blob/fix/motion-convergence/packages/ozton/examples/programmed.rs) for full setup.

## Choose an estimator

Odometry integrates measured motion. It is a good starting point and needs no walls. EKF combines observations with modeled uncertainty. MCL maintains a bounded set of particles and uses associated range observations to correct translation; it relies on an accurate surveyed map and correctly mounted sensors. DSR is an explicit stationary distance reset, not a continuous motion controller.

Only model real, fixed surfaces. A moving game object is not a reliable wall. Distance samples are timestamped and may be rejected when stale, occluded or inconsistent. Range corrections can be suspended without stopping raw sensor logging.

## Start and stop localization

Application-owned robots can return LocalizationWorker::start from OztonCompete::start_localization. Choose LocalizationSchedule::Autonomous to run only during autonomous, or Always to also update while driving and disabled. The builder has with_localization_schedule for the same purpose.

Register the actual starting pose with drivetrain.localization_mut().set_pose(pose), and check its boolean result. The worker needs fresh samples before feedback becomes available. Pausing invalidates the old baseline. Do not use an unregistered or stale pose for field motion.

In an open calibration area, select Estimator::Odometry and turn off external corrections using set_corrections_enabled(false). set_motion_bounds can provide a virtual boundary without replacing the surveyed wall map. Re-enable the correct estimator and register the physical starting pose when returning to a walled field.

A fixed wall is a single `Surface::new(start_xy, end_xy, height)` with a continuous vertical face from the floor to `height`. For an elevated fixture, use `.with_base_height(base)` on the same type. Sensor-height masks are resolved at localizer construction, so rays above or below the face cannot contribute to MCL or DSR. Planar ray-casting primitives still accept precomputed masks.


## Velocity estimation

Velocity is a separate state from absolute field pose. MCL and DSR corrections
are never differentiated as if the chassis had physically moved. A single
timestamp-aware linear Kalman estimator supplies forward, lateral and yaw
rates to PID, stationary DSR gating, diagnostics and motion safety.

Each linear axis maintains velocity v and accelerometer bias b. With a calibrated
acceleration input a, the state prediction is v' = v + (a - b) dt, b' = b.
The covariance prediction is F P F^T + Q, with F = [[1, -dt], [0, 1]].
Without acceleration, velocity follows a random-walk process. The rate
measurement update uses H = [1, 0], K = P H^T / (H P H^T + R), and the
Joseph covariance update. For an encoder increment d, z = d/dt and
R = 2 sigma_position²/dt². Direct motor velocity is preferred when drive
encoders provide forward travel; it replaces, rather than double-counts,
the correlated encoder delta. An independent forward-projected pod retains
priority. Gyro yaw rate is preferred to differentiated heading. All angles
and angular rates are CCW radians. A rotating chassis frame includes the
transport term -omega cross v in its acceleration prediction.

Use LocalizationBuilder::with_velocity_config to specify measured noise
parameters. Default process/measurement noise is a conservative initial
model, not a measured specification. Estimate encoder position and direct
rate noise at rest and at steady speed, and gyro noise at rest, from logs.
The accelerometer channel accepts SI, gravity-compensated chassis-frame
acceleration at the origin. The V5 flat-mount adapter is explicitly opt-in
via InertialSensorTracking::with_planar_acceleration(true). Verify axes
with a static tilt, forward/reverse acceleration and stationary drift test
before enabling it. VelocityConfig::acceleration_axes independently selects
validated right and forward channels; nonzero mount offsets require rotational acceleration
compensation and are not enabled by this adapter. VEX reports acceleration
in g and gyro rate in degrees/s:
https://vexide.dev/docs/inertial-sensor/ .

The estimator resets on a new surveyed pose, invalidated tracking or a
configuration change. Raw odometry displacement still drives pose integration.
No motion session maintains a separate packet-spike workaround.

DistanceTracker sources do not block Tracking::new on optional range readiness.
Normal capture checks the device and requires a newly observed packet edge before
publishing a timestamped range. Disconnected sensors keep their configured slots
and can recover on later captures. The required IMU calibration still blocks
startup until it succeeds; explicit Distances::initialize remains available for
applications that intentionally wait on range readiness.


## Incomplete maps and efficient beam updates

A finite range is eligible only if its modeled ray reaches an eligible surface
within the sensor range, and passes normal incidence, edge, age and residual
gates. An open ray is not a measurement of a virtual boundary. The virtual
motion perimeter is separate from the surveyed reflecting map.

Within MCL, let H be the particles with finite in-range predicted hits and
L_i the robust Gaussian-plus-outlier likelihood. Their conditional mean
likelihood is sum_H(w_i L_i) / sum_H(w_i). Multiply modeled weights by
L_i divided by this mean; leave unmodeled weights unchanged. Thus the modeled
and unmodeled groups retain their prior probability mass relative to each other.
A beam with no modeled mass is skipped. No correction is returned if every beam
is skipped. This is a missing-geometry model, not a negative free-space sensor.

The common translation fit accepts all coherent observations or a unique
leave-one-out coherent set of at least two readings; ambiguous subsets are
rejected. Explicit stationary DSR continues to require a consistent fit of its
collected readings. Rank-one fits use the actual principal eigenvector.

Distance-only particle rays use prepared segment vectors and squared parallel
thresholds, iterate the height-mask bits, and compare signed cross-product
numerators before division. Only closer valid intersections require division.
Fixed-heading updates share a rotated beam origin offset and direction across
all particles. Free-heading updates cache each particle orientation across
beams. Translation-only moments avoid circular-heading computations; the
Box-Muller sampler reuses both independent normal variates. These are algebraic
work reductions, not changes to the range noise or particle count.
