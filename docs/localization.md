# Configure localization

Start with geared drive encoders and a calibrated IMU. Add a forward tracking
pod or lateral pod where wheel slip is significant. Supply calibrated pod
travel in meters with `Sensors::with_forward_wheel` or
`Sensors::with_lateral_wheel`. Set the corresponding offset in `TrackingSetup`.
A forward pod's offset is its X-right position from CC. A lateral pod's offset is
its Y-forward position from CC. Lateral travel is positive right (+X). Set `require_lateral` for robots that rely on measured sideways travel.
Use `TrackingWheel::from_dimensions` to pass the diameter and signed offset
as `Length` values.

Each tick uses one sensor snapshot. A recovered source needs a fresh baseline
before it contributes increments. Differential odometry can fall back to drive
encoders when a pod drops out, or wheel-based heading when the IMU is missing
and both sides remain observable.

## Add field references

Register stationary reflecting segments with `with_map` and name the devices
with `with_range`. A mount uses X right, Y forward, and a CCW beam bearing from +Y.
Use `RobotPoint::new(right, forward, up)` for the installed position and
`horizontal_range_mount(bearing)` to create its planar beam mount.
Use a stable wall or fixture. Pass occlusion bits to `Chassis::sample` from
`AutonomousControls::sample` when a mechanism blocks a beam. With the stock
adapter, supply those bits through `Mechanisms::occluded_ranges`. Use
`ConfiguredChassis::range(name)` and `RangeId::mask` to get a sensor's bit.

- **DSR** uses one wall to correct its normal axis, or two independent wall
  observations to correct both. Use `Command::DistanceReset` when the robot
  can see the intended surfaces.
- **EKF** uses wheel and IMU prediction with gated GPS position or wall
  measurements. Supply noise values that match the robot and field.
- **MCL** uses a bounded particle cloud around the known starting pose and
  gated wall ranges. It publishes bounded position corrections while heading
  follows the odometry sources.
- **Contact** sets a known pose after the contact condition and heading settle.
  Use contact geometry that constrains the pose you set.

Choose the estimator on `TrackingSetup`. Begin with odometry and run EKF or MCL
with `shadow = true` to compare its candidate pose against the odometry baseline.
Verify repeated runs on the actual field before enabling corrections.

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
`offset_x * yaw` and lateral travel `-offset_y * yaw`; odometry subtracts these
lever-arm terms before integrating CC travel. An IMU supplies angular changes. A zero configured
track width explicitly disables wheel-heading and single-side fallback when that
geometry is unknown; it requires valid IMU and both drive encoder readings.

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

All filter positions and innovations are X-right/Y-forward; there is no hidden
axis swap at the robot boundary. The geometry
helper returns `direction(0) = (0, 1)`. Field poses transform body mounts by an ordinary CCW
rotation about CC. The geometric rotation operator is separate from a forward vector.

The localizer's external updates are translational. EKF position updates preserve
IMU/odometry yaw using a position-only gain and a Joseph covariance update. MCL
conditions particle yaw on the independently tracked heading before weighting beams,
so it cannot explain position errors with a yaw different from the published pose.
Sparse beams still correct only their observed translation axes.

When GPS is enabled, set `Sensors::set_field_origin` to the same surveyed origin
and initial orientation used for maps/targets. `GpsTrackingConfig::field_origin`
provides the same registration for standalone GPS tracking. Raw GPS uses the centered
field survey; pose `in_frame(origin)` performs the explicit translation and rotation.
Override configures this registration automatically when a field/map is registered.
