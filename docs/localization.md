# Configure localization

Start with geared drive encoders and a calibrated IMU. Add a forward tracking
pod or lateral pod where wheel slip is significant. Supply calibrated pod
travel in meters with `Sensors::with_forward_wheel` or
`Sensors::with_lateral_wheel`. Set the corresponding offset in `TrackingSetup`.
A forward pod's offset is positive left. A lateral pod's offset is positive
forward. Set `require_lateral` for robots that rely on measured sideways travel.

Each tick uses one sensor snapshot. A recovered source needs a fresh baseline
before it contributes increments. Differential odometry can fall back to drive
encoders when a pod drops out, or wheel-based heading when the IMU is missing
and both sides remain observable.

## Add field references

Register stationary reflecting segments with `with_map` and name the devices
with `with_range`. A mount uses forward X, left Y, and a robot-relative beam
heading. Use a stable wall or fixture, and pass occlusion bits through
`Mechanisms::occluded_ranges` when a mechanism blocks a beam. A sensor's
`RangeId::mask` produces its bit.

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
