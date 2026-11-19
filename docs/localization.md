# Configure localization

Start with geared drive encoders and a calibrated IMU. Add a forward tracking
pod or lateral pod where wheel slip is significant. Supply calibrated pod
travel as `Length` with `Sensors::with_forward_wheel` or
`Sensors::with_lateral_wheel`. Set the corresponding offset in `TrackingSetup`.
Measure from [The Origin](coordinates.md). Use `RobotPoint::right` for a
forward pod and `RobotPoint::forward` for a lateral pod. Lateral travel is
positive to the right.
Set `require_lateral` for robots that rely on measured sideways travel.
Use `TrackingWheel::new` to pass the diameter and signed offset
as `Length` values.

Each tick uses one sensor snapshot. A recovered source needs a fresh baseline
before it contributes increments. Differential odometry can fall back to drive
encoders when a pod drops out, or wheel-based heading when the IMU is missing
and both sides remain observable.

## Add field references

Pass stationary reflecting segments as the map and pair each range device
with its mount. A mount uses X right, Y forward, and a CCW beam bearing from +Y.
Use `RobotPoint::new(right, forward, up)` for the installed position and
`horizontal_range_mount(bearing)` to create its planar beam mount.
Use a stable wall or fixture. Pass occlusion bits to `Chassis::sample` from
`AutonomousControls::sample` when a mechanism blocks a beam. Use
`ConfiguredChassis::range(name)` and `RangeId::mask` to get a sensor's bit.

- **Distance sensor reset (DSR)** uses one wall to correct its normal axis, or two independent wall
  observations to correct both. Use `Command::DistanceReset` when the robot
  can see the intended surfaces.
- **Extended Kalman filter (EKF)** uses wheel and IMU prediction with gated GPS position or wall
  measurements. Supply noise values that match the robot and field.
- **Monte Carlo localization (MCL)** uses a bounded particle cloud around the known starting pose and
  gated wall ranges. It publishes bounded position corrections while heading
  follows the odometry sources.
- **Contact** sets a known pose after the contact condition and heading settle.
  Use contact geometry that constrains the pose you set.

Use `ozton::tracking::localizer::Tracking` to compose a localizer with the
estimator and any range devices used outside localization. Construct an
`Ekf` or `Mcl` independently and attach it at assembly.

```rust,ignore
let tracking = Tracking::new(localizer)
    .with_distances(Distances::new([front, left]).expect("supported range count"))
    .with_mcl(Mcl::new(start_pose, particle_config).expect("valid particle config"));
```

Use `tracking.distances.latest(index)` to inspect the same packet used by
localization. Calibration uses this path without polling the device again.
If only the localizer needs the readings, omit `with_distances` and pass
observations to `tracking.localizer.update(snapshot, observations)` from the
owner of the sensors. Map geometry and wheel offsets are constructor inputs.

Begin with odometry and run EKF or MCL with `shadow = true` to compare its
candidate pose against odometry. Use the legacy `Estimator` selection only
when the running application needs to change algorithms.

## Height-aware DSR and horizontal odometry

`RangeSensorConfig::surface_mask` selects eligible static surfaces for each range
sensor, for example only walls tall enough to intersect its horizontal beam.
Relative range validation remains independent of map eligibility. DSR reports
`Update::distance_reset_axes` as 0, 1 or 2; a single observed wall normal must not
be described as a full XY position fix. `cancel_distance_reset()` clears a pending
reset after an optional command timeout. MCL ray casts each beam against its own eligible surface mask, just like DSR/EKF.
An ineligible nearer low wall cannot attract a high beam's particle likelihood.

For a differential drive with one horizontal tracking wheel, use drive encoders
for forward travel and the wheel for `Snapshot::lateral`, positive right. Both the
localizer and `wheeled::TrackingWheel` use the same Y-forward lateral offset.
Forward wheels use their X-right offset. Pure CCW rotation produces forward travel
`offset_x * yaw` and lateral travel `-offset_y * yaw`. Odometry subtracts these
lever-arm terms before integrating travel at The Origin. An IMU supplies angular
changes. A zero track width disables wheel-heading and single-side fallback.
With that setting, odometry needs valid IMU and both drive encoder readings.

## Rest, consistency and pose trust

DSR waits for planar speed <= 0.025 m/s and angular speed <= 0.06 rad/s over 120 ms,
then collects fresh packets over at least 45 ms. Motion restarts the window. All
eligible constraints enter a weighted fit. Residuals exceeding three normal-axis
standard deviations (with a 25 mm floor) reject the fit. One-axis fits preserve the
unobserved tangent. The same redundant-beam consistency gate precedes EKF/MCL updates.
Rejected MCL candidates do not change the filter's stored posterior.

`Localizer::pose_trust()` starts `Unregistered`. Explicit `set_pose` registers a
surveyed start/contact pose. A motion/time fault latches `NeedsRelocalization` and
discards the unobserved interval. Neither sensor reconnection nor a partial DSR
restores `Trusted`. Absolute runner commands check trust every tick. Relative-only
commands can remain usable under their own bounds. `set_estimator` changes filters
without changing trust. MCL uses a local cloud; it does not recover a kidnapped robot.

All available motors in a geared side contribute normalized wheel increments.
`EncoderConsensus` uses median travel and rejects disagreement, encoder jumps and
nonfinite data. A new/rejoined motor receives its own baseline. Common-mode wheel
slip remains unobservable without an independent pod or valid external references.

All filter positions and innovations use X right and Y forward. The geometry
helper returns `direction(si(0.0)) = (0, 1)`. A field pose transforms a body mount by
rotating its offset counterclockwise about The Origin and translating it to
the robot's field position.

The localizer's external updates are translational. EKF position updates preserve
IMU/odometry yaw using a position-only gain and a Joseph covariance update. MCL
conditions particle yaw on the independently tracked heading before weighting beams,
so it cannot explain position errors with a yaw different from the published pose.
Sparse beams still correct only their observed translation axes.

## Register GPS

When GPS is enabled, set `Sensors::set_field_origin` to the same surveyed origin
and initial orientation used for maps/targets. `GpsTrackingConfig::field_origin`
provides the same registration for standalone GPS tracking. Raw GPS uses the centered
field survey; pose `in_frame(origin)` performs the explicit translation and rotation.
