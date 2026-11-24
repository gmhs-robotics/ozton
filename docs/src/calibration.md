# Calibrating a robot

A controller cannot correct measurements that are consistently wrong. First verify motor directions, actual gear ratios, effective wheel diameter, track width, sensor orientation and field dimensions.

## Establish a baseline

Start with odometry and IMU in an unobstructed area. Register a measured starting pose, command short forward and reverse moves, and measure the actual endpoint. Repeat turns in both directions. Then try longer commands and record the stop distance. Keep the mechanism stowed and have someone at the disable control.

Do not tune PID gains from an estimator that is simultaneously changing its position because of unverified range corrections. Run raw wheel-speed step tests to measure acceleration, motor response and braking. Tune physical velocity controllers separately from normalized voltage controllers.

## Verify localization separately

In a walled area, measure the sensor mounts and wall dimensions, enable MCL, and compare published pose to an independently measured pose. Run DSR while stationary with fresh observations. Investigate rejected or stale observations instead of forcing the filter to accept them.

## Check the result

Record the initial configuration and battery voltage for each trial. Compare time to settle, endpoint error and overshoot across repeated runs. Use the [logging and profiling guide](recording.html) to identify timing gaps before changing PID gains. A simulated test passing does not establish physical accuracy.
