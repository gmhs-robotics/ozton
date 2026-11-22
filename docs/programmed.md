# Build a programmed robot

Assemble your controller, drivetrain, tracking, mechanisms, routes, and Brain
display in `main.rs`. Pass each port to its owner once. Use
`.with_mechanisms(Mechanisms { intake, lift })` for named fields, then access
them as `robot.mechanisms.lift` in either mode. Omit this step for a drive-only
robot.

```rust,ignore
{{#include ../packages/ozton/examples/programmed.rs:assemble}}
```

Select a route while disabled. `compete()` stops outputs on mode changes and runs
the selected async routine when autonomous begins.

## What happens each driver tick

1. Ozton reads one controller snapshot and refreshes lightweight inputs.
2. Both the driver callback and autonomous routes use
   `async fn(robot: &mut Robot) -> Result<(), ProgramError>`. The driver reads its
   snapshot with `robot.driver_input()` and its elapsed time with
   `robot.driver_elapsed()`. Apply a chassis command with `robot.drive_driver(...)?`.
3. Ozton calls `Program::update_mechanisms` after the driver callback.
   Autonomous motion and managed waits call the same mechanism hook.
4. Ozton polls optional tracking and diagnostics within the driver tick budget.
   A controller disconnect or failed drive write stops both drive and mechanisms.

Put the driver mapping and async route functions in one program module. Keep
device logic and coordination in mechanism modules. Store physical limits in
the mechanism code so both modes enforce them. Apply driver-only speed shaping
when you build the driver command. Implement `stop_mechanisms` to bring custom
actuators to rest and discard stale goals.

## Run a route from a button

A future that borrows the whole robot holds that borrow until it completes.
For a routine that must run alongside steering, let the driver callback start
a task using a mechanism-owned command handle. It can use ordinary
`vexide::time::sleep(...).await` while the competition loop continues polling
driver input. Store the task and cancel it in the program's stop hook and
when manual inputs take over.

Return errors with `?` in both modes. Ozton reports failures to the Brain and
runs the mode's stop or release path. A route can return the program's error or
any error convertible to it.

## Choose a drivetrain

`DifferentialConfig::build` sets up a differential chassis with known wheel
diameter, free speed, track width, and maximum wheel speed. Its `TrackingSetup`
owns the IMU, range mounts, and field map. Keep the robot still during IMU
calibration. Await setup from `main` before calling `compete()`. Use [mounted distance trackers](localization.md#add-field-references)
and enter each sensor once in wiring order.

The configured chassis supplies the feedback for `RobotMotion` automatically.
Its default controller uses native motor velocity. `WheelControl::Voltage`
uses PID and feedforward in volts, capped by the motor rating.

A different drivetrain implements `ProgramDrive` with its own driver and
autonomous command types. Use `Auton::drive_for` for bounded custom commands and
`Auton::wait_until` for mechanism predicates. Both share the selected route's
deadline and stop on competition cancellation.

## Complete example

```rust,ignore
{{#include ../packages/ozton/examples/programmed.rs}}
```

## Drive assists

Differential and Mecanum both support heading hold, anti-tip, deadband, boost,
slew and measured-load slowdown. Pass heading and fore-aft tilt from tracking,
and pass mechanism position with `ArcadeInput::with_load`. The stock drive
applies `with_slowdown(full_load, minimum_scale)` to driver output, including
boost. Missing load feedback uses the minimum scale. Programmed wheel targets
use the configured physical wheel-speed limit.

## Assemble tracking from sensors

Build the drivetrain and tracker separately in `main`. To track drive-wheel
travel, use `Differential::new(left, right).with_wheel_diameter(diameter)`.
Each side accepts `(Motor, (driver_teeth, driven_teeth))` entries. Use
`WheelGearing::new(driver_teeth, driven_teeth).then(driver_teeth, driven_teeth)`
for compound gearing. The model converts each motor's travel and speed to the
wheel before combining healthy encoders on that side.

```rust,ignore
let drivetrain = Differential::new(left, right)
    .with_wheel_diameter(diameter)?
    .with_track_width(track_width)?
    .with_maximum_wheel_speed(maximum_wheel_speed)?
    .with_antitip(anti_tip)
    .with_headinghold();
let tracking = TrackingBuilder::new()
    .with_drive_encoders(DriveEncoderUse::ForwardAndHeading)
    .with_imu(imu_port, [ImuUse::HeadingCorrection, ImuUse::Orientation])
    .with_wheels([MountedWheel::from_rotary(
        lateral_encoder, diameter, position!(0 in, -1 in, 0 in),
        Angle::new::<degree>(-90.0), None,
    )])
    .with_dsr(wall_sensors, [DsrUse::Dsr, DsrUse::Ekf]);
let robot = RobotBuilder::new()
    .with_drivetrain(drivetrain)
    .with_tracking_from_drive(async |drive| {
        let tracker = tracking.build_for_drive(drive, |mounts| {
            Localizer::new_owned(config, start_pose, field, mounts.to_vec())
        }).await?;
        Some(tracker)
    })
    .await?;
```

`DriveEncoderUse::Forward` averages both sides. `ForwardAndHeading` also
allows wheel-derived yaw using the drivetrain's own track width. A
differential drive pair observes forward travel and yaw. Bearings are
counterclockwise from forward, so a right-positive lateral pod has a
bearing of −90°. Use `OdometryConfig::for_mounted_wheels` to set motion limits.
Fully forward-aligned layouts may assume zero lateral travel.

Tracking uses valid IMU yaw first. It prefers a complete fit from independent
pods, then includes drive encoders with lower weight where needed. A lateral
pod only measures motion along its own axis. Range corrections constrain
accumulated drift when the observation passes the field, confidence, and
freshness gates. Geometric resets collect data at rest. IMU diagnostics
are read by the enabled diagnostics sink without a tracking role.

Repeat `with_dsr` to register more groups. The roles control which sensors feed
geometric resets or the active filter. The `Localizer` configuration selects
Odometry, EKF, or MCL. `set_estimator` switches filters at runtime. One distance
reading can serve multiple roles, and sensor indices follow registration order.

In the competition update, call
`tracking.poll_with_drive(mode, occluded, &mut robot.drivetrain)`. The tracker
borrows the drivetrain for one sample. For tracking pods alone, use
`tracking.poll_wheels(mode, occluded)`. Custom drive wrappers can implement
`DriveEncoders` to expose their calibrated left/right travel.
