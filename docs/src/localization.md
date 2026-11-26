# Tracking and localization

Tracking reads devices. Localization interprets the readings and estimates the robot's field pose. Keeping these separate means a different estimator does not require rewiring your robot.

## Assemble a tracked drivetrain

Create the motor groups once with shared_motors!, then use Differential::from_shared and DifferentialTracking::from_shared (or its calibrated constructors) so motor commands and encoders refer to the same devices. Give DifferentialTracking the actual wheel diameter, track width and gear ratios in motor order. An IMU and a lateral tracking wheel can improve the estimate.

Distance trackers already carry a default acceptance gate, zero bias and 25 mm measurement noise. All localization roles are enabled by default. Restrict an individual sensor with `.with_roles(RangeRoles::DSR)` or combine roles with `.union(...)`; use `RangeRoles::NONE` for relative feedback without localization corrections. Construct them directly with `DistanceTracker::at_mount(sensor, mount)`; add `.with_bias(...)`, `.with_sigma(...)` or `.with_gate(...)` on that tracker only when calibration requires it.

Use the role-based tracking builder to assemble mounted sources. This fragment assumes the devices, field and calibrated geometry are already available:

```rust
use ozton::{
    drivetrain::Drivetrain,
    localization::Localization,
    tracking::Tracking,
};

let tracking = Tracking::builder()
    .with_drive(drive_encoders)
    .with_extra((lateral_wheel,))
    .with_orientation(imu)
    .with_ranges([front_range])
    .build()
    .await?;
let localization = Localization::builder()
    .with_surface_map(field)
    .build(&tracking)
    .expect("check tracking geometry");
let drivetrain = Drivetrain::builder(model.with_footprint(footprint))
    .with_tracking(tracking)
    .with_localization(localization)
    .build();
```

The source types automatically determine the tracking capabilities, regardless of tuple order. A calibrated differential encoder source grants `HasDriveWheels`; an orientation source grants `ReadHeading` or `ReadTilt` when it implements the corresponding interface; at least one statically declared range source grants `HasRange`. No sensor indices are needed in application code. Custom sources declare their interfaces with `TrackingSource::Capabilities`. Duplicate differential sources and more than eight statically declared range slots are rejected at compile time; geometry is validated during assembly. The tuple constructor `Tracking::new` is also available. Named roles make the selected drive and orientation explicit.

The fragment is not a complete robot; field, model, footprint and sensor objects are your measured hardware. Pass the assembled tracking configuration directly to the localization builder's `build` method. Odometry is the default; selecting MCL or EKF with `with_estimator` adds a compile-time range capability requirement and a startup check for visible mapped surfaces. For a holonomic chassis, use mounted wheel projections and disable differential drive encoders when they are not appropriate.

## Survey a displaced rotation center

The robot-local origin is the effective center of rotation on the floor. The
original CAD survey need not use that point. Rebase every mounted sensor and
the physical footprint with the same constant frame, before constructing
tracking or localization:

```rust
use ozton::{position_xyz, position_xyzh, tracking::mount::MountFrame};

const FRAME: MountFrame =
    MountFrame::new().with_origin(position_xyz!(0 in, -1 in, 0 in));
const IMU_MOUNT: (ozton::units::Vec3<ozton::uom::si::f64::Length>, ozton::uom::si::f64::Angle) =
    FRAME.mount(position_xyzh!(0 in, 0.5 in, 3 in, 0 deg));
assert_eq!(IMU_MOUNT.0.forward, ozton::qty!(1.5 in));
```

Pass `FRAME.mount(surveyed_mount)` to each mounted wheel, range tracker and
IMU, and `FRAME.footprint(surveyed_footprint)` to the drivetrain model.
Use `FRAME.point` for other surveyed body points. Differential drive encoders
assume an origin on the longitudinal midline; choose the effective axle's
forward coordinate, keeping X and Z at zero. The frame is a construction-time
translation, not a runtime pose reset. Field walls, obstacles and headings
do not move; all pose targets and registrations refer to the new robot origin.

To calibrate the axle location, log signed sideways wheel travel and unwrapped
CCW angle during isolated spins in both directions.
`tracking::mount::estimate_forward_pivot(tracker_forward, travel, yaw)`
returns the pivot in the same survey as the supplied tracker coordinate.
Average repeatable estimates and update the frame; do not change the physical
tracker measurement to absorb slip. The effective pivot can vary with load and
surface, so verify the model on the robot.

## Choose an estimator

Odometry integrates measured motion. It is a good starting point and needs no walls. EKF combines observations with modeled uncertainty. MCL maintains a bounded set of particles and uses associated range observations to correct translation; it relies on an accurate surveyed map and correctly mounted sensors. DSR is an explicit stationary distance reset, not a continuous motion controller.

Only model real, fixed surfaces. A moving game object is not a reliable wall. Distance samples are timestamped and may be rejected when stale, occluded or inconsistent. Range corrections can be suspended without stopping raw sensor logging.

## Start and stop localization

Application-owned robots can return LocalizationWorker::start from OztonCompete::start_localization. Choose LocalizationSchedule::Autonomous to run only during autonomous, or Always to also update while driving and disabled. The builder has with_localization_schedule for the same purpose.

Register the actual starting pose with drivetrain.localization_mut().set_pose(pose), and check its boolean result. The worker needs fresh samples before feedback becomes available. Pausing invalidates the old baseline. Do not use an unregistered or stale pose for field motion.

In an open calibration area, select Estimator::Odometry and turn off external corrections using set_corrections_enabled(false). Optional physical contact registration dimensions are supplied through `with_contact_bounds`; motion geometry is separate. Re-enable the correct estimator and register the physical starting pose when returning to a walled field.

A fixed wall is a single `Surface::wall(start_xy, end_xy, height)` with a continuous vertical face from the floor to `height`. For an elevated fixture, use `.with_base_height(base)` on the same type. Sensor-height masks are resolved at localizer construction, so rays above or below the face cannot contribute to MCL or DSR. Planar ray-casting primitives still accept precomputed masks.


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

Use LocalizationBuilder::with_velocity_estimator to specify measured noise
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

Wheel odometry skips underdetermined pod-only fits, solves the combined system
once (full-rank or partial as appropriate), and avoids yaw normalization when
heading is independently known. MCL logs distinguish lack of a new range packet
from missing modeled surfaces, residual and other measurement rejections. The
fraction of ticks with no new packet is not an estimator failure rate.


## V5 sensor range and distance-dependent accuracy

The V5 Distance Sensor (276-4852) specifies a 20–2000 mm measurement
interval, approximate ±15 mm accuracy below 200 mm and approximately 5%
above 200 mm:
https://kb.vex.com/hc/en-us/articles/360050696511-Using-the-Distance-Sensor-with-VEX-V5

The hardware interval is enforced on raw packets before bias correction;
corrected observations and predicted nearest hits must also lie inside the
application gate intersected with the hardware interval. The default gate
retains its conservative 50 mm lower limit. Configure a tighter upper range
directly or use RangeSensorConfig::limit_accuracy / DistanceTracker::
with_maximum_range_error (e.g. 50 mm implies at most 1 m).
The short-range 10/63 unavailable-confidence sentinel documented by PROS
(https://pros.cs.purdue.edu/v5/api/cpp/distance.html) is accepted only
at or below 200 mm; genuine long-range low confidence remains rejected.

The noise model uses max(calibrated sigma, 15 mm, 0.05 * range).
This continuous conservative engineering envelope is treated as a standard
deviation, not a statistical guarantee by VEX. For associated constraints use
the farther of the measured and predicted distances, projecting the resulting
sigma along the wall normal. MCL additionally respects its configured noise
floor, evaluates a constant per-beam likelihood width (avoiding particle-
dependent normalization bias), and ignores predicted hits outside either
range limit. EKF, consensus and stationary DSR use the same sensor noise
model. DSR remains a geometric reset, not a probabilistic bound on accuracy;
tighten the gate for precision-sensitive resets. The maximum MCL residual
remains an independent association ceiling, not the hardware range.

## Optical height versus physical obstacles

Each horizontal beam is assigned an intersection mask of its explicit surface
selection and faces whose vertical interval [base, base + height) contains its
surveyed mounting height. This is computed once at Localizer construction, not
per particle. Sensors with an empty mask are excluded from MCL, EKF and DSR;
they remain available for raw or relative-distance feedback. Heights assume
a level chassis and horizontal center rays, not a full optical cone model.

Start with an empty `SurfaceMap`. Add physical walls and convex extruded
polygons using the shared `Surface` constructors. Only these explicitly
registered surfaces are used for optical localization. Use
`MotionMap::from_localization(&map)` to reuse their XY footprints for optional
host-side clearance checks. Additional motion surfaces do not affect range
association, even if their XY location intersects a sensor ray. See
[Surface maps](surface-maps.md) for examples. Validate geometry registration
and localization construction results.

Range consensus checks both each normalized residual and their combined squared
sum against the residual degrees of freedom. This prevents contradictory
redundant readings from passing merely because the distance-dependent noise
increases their individual tolerances. The existing absolute residual floor
is preserved; this is an engineering consistency gate, not an exact chi-square
confidence test.
