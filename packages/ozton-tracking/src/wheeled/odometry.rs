//! Sampling and integration loop for the background wheeled tracker.
use super::*;

impl WheeledTracking {
    pub(super) fn compute_raw_heading<G: Gyro, T: RotarySensor>(
        gyro: Option<&G>,
        parallel_wheels: Option<(&TrackingWheel<T>, &TrackingWheel<T>)>,
    ) -> Result<Angle, HeadingError<T>> {
        assert!(
            gyro.is_some() || parallel_wheels.is_some(),
            "No gyro or wheeled tracking sensors provided to compute_heading."
        );

        // Try to get a reading of the robot's heading from the gyro. We should only do this if the
        // gyro hasn't returned a port-related error before (flagged by the `gyro_invalid`
        // variable). If it has, the gyro has no chance of recovery and we should fallback to
        // wheeled heading calculation.
        let gyro_rotation = gyro.as_ref().map(|gyro| gyro.heading());

        // Compute the unbounded robot orientation in radians. In the case of the gyro, this
        // actually is bounded to [0, TAU] already due to how gyro heading works, but this will be
        // wrapped to [0, TAU] regardless later either way.
        let raw_heading = Angle::from_radians(if let Some(Ok(gyro_heading)) = gyro_rotation {
            // gyros' frame of reference is NED (Z-Down), meaning heading increases as the robot
            // turns clockwise. We don't want this, since it doesn't match up with how the unit
            // circle works with cartesian coordinates (what we localize in), so we need to convert
            // to a CCW+ angle system.
            gyro_heading.as_radians()
        } else if let Some((left_wheel, right_wheel)) = parallel_wheels {
            // Distance between the left and right wheels.
            let track_width = right_wheel.offset.value - left_wheel.offset.value;

            // Nothing we can use if either of these disconnects, so all we can do is wait for them
            // to reconnect. Seriously, fix your wiring!
            let left_travel = left_wheel
                .travel()
                .map_err(|err| HeadingError::RotarySensor(err))?;
            let right_travel = right_wheel
                .travel()
                .map_err(|err| HeadingError::RotarySensor(err))?;

            (right_travel.value - left_travel.value) / track_width
        } else if let Some(Err(_)) = gyro_rotation {
            // gyro failed and we have no viable sensors to determine heading. Nothing we can do to
            // recover from this.
            return Err(HeadingError::Imu(None));
        } else {
            unreachable!() // handled by the assertion at the top of this function
        });

        match gyro_rotation {
            // gyro failed but we have a wheeled heading source to fall back to.
            Some(Err(_)) => Err(HeadingError::Imu(Some(raw_heading))),
            _ => Ok(raw_heading),
        }
    }

    // MARK: Task

    #[allow(clippy::too_many_arguments)]
    pub(super) async fn task<
        T: RotarySensor,
        U: RotarySensor,
        G: Gyro,
        const NUM_FORWARD: usize,
        const NUM_SIDEWAYS: usize,
    >(
        forward_wheels: [TrackingWheel<T>; NUM_FORWARD],
        sideways_wheels: [TrackingWheel<U>; NUM_SIDEWAYS],
        mut gyro: Option<G>,
        data: Rc<RefCell<TrackingData>>,
        parallel_forward_indicies: Option<(usize, usize)>,
        mut prev_forward_wheel_data: [Result<(f64, f64), <T as RotarySensor>::Error>; NUM_FORWARD],
        mut prev_sideways_wheel_data: [Result<(f64, f64), <U as RotarySensor>::Error>;
            NUM_SIDEWAYS],
        mut prev_raw_heading: Angle,
    ) {
        let mut prev_time = Instant::now();
        let mut align_wheel_heading = false;

        loop {
            sleep(Motor::WRITE_INTERVAL).await;

            let mut data = data.borrow_mut();

            let forward_wheel_data = forward_wheels.each_ref().map(|wheel| {
                wheel
                    .travel()
                    .map(|travel| (travel.value, wheel.offset.value))
            });
            let sideways_wheel_data = sideways_wheels.each_ref().map(|wheel| {
                wheel
                    .travel()
                    .map(|travel| (travel.value, wheel.offset.value))
            });

            if forward_wheel_data.iter().any(Result::is_err)
                || sideways_wheel_data.iter().any(Result::is_err)
            {
                data.ready = false;
                prev_forward_wheel_data = forward_wheel_data;
                prev_sideways_wheel_data = sideways_wheel_data;
                prev_time = Instant::now();
                continue;
            }

            // Calculate absolute robot orientation (heading).
            //
            // This can be done in two possible ways - Either using a gyro (if it is available and
            // actually working) or through the use of two parallel forward trackers. The former is
            // generally far more reliable and isn't prone to wheel slip.
            let mut heading_source_changed = false;
            data.raw_heading = match Self::compute_raw_heading(
                gyro.as_ref(),
                parallel_forward_indicies.map(|(left_index, right_index)| {
                    (&forward_wheels[left_index], &forward_wheels[right_index])
                }),
            ) {
                // Cool
                Ok(raw_heading) => {
                    if align_wheel_heading {
                        data.heading_offset = Self::aligned_heading_offset(
                            data.heading_offset,
                            prev_raw_heading,
                            raw_heading,
                        );
                        prev_raw_heading = raw_heading;
                        align_wheel_heading = false;
                        heading_source_changed = true;
                    }
                    raw_heading
                }

                // We got an error from the gyro, which means it likely disconnected. Once a gyro
                // disconnects it will reclibrate upon regaining power, which will mess tracking up
                // badly, so we need to stop using it in this case and switched to a wheeled method
                // of determining heading.
                Err(HeadingError::Imu(raw_wheel_heading)) => {
                    data.ready = false;
                    if raw_wheel_heading.is_some() {
                        gyro = None;
                    }

                    // Use the backup wheeled heading value in the gyro failed.
                    if let Some(raw_wheel_heading) = raw_wheel_heading {
                        data.heading_offset = Self::aligned_heading_offset(
                            data.heading_offset,
                            prev_raw_heading,
                            raw_wheel_heading,
                        );
                        prev_raw_heading = raw_wheel_heading;
                        heading_source_changed = true;
                        raw_wheel_heading
                    } else {
                        // If no backup heading is available, that means we have no means of
                        // determining heading in the future (because we have no parallel forward
                        // trackers), meaning there's no point in continuing this task. Womp womp.
                        return;
                    }
                }

                // Occurs if both the gyro failed and the backup heading source failed.
                Err(HeadingError::RotarySensor(_)) if gyro.is_some() => {
                    data.ready = false;
                    gyro = None; // No more gyro :(
                    align_wheel_heading = true;
                    continue;
                }

                // One of the tracking wheels failed and we don't have a gyro, so just wait for it
                // to reconnect I guess.
                _ => {
                    data.ready = false;
                    continue;
                }
            };

            // Change in raw heading from the previous loop iteration.
            let delta_heading = (data.raw_heading - prev_raw_heading).wrapped_half();

            // Average between the current and previous heading reading used conversion between
            // global and local coordinate displacements.
            //
            // No need to wrap since we only plug this into trig functions.
            let avg_heading =
                (data.raw_heading - (delta_heading / 2.0) + data.heading_offset).wrapped_full();
            prev_raw_heading = data.raw_heading;

            let mut local_displacement: Vec2 = Vec2::default();
            let yaw = delta_heading.as_radians();
            // One division per tick, shared by all wheels. Removing the wheel's
            // rotational travel before scaling also avoids dividing tiny deltas.
            let chord_scale = if yaw == 0.0 {
                1.0
            } else {
                2.0 * (delta_heading / 2.0).sin() / yaw
            };

            // MARK: Sideways Wheels

            // Doing all the stuff with sideways trackers in this block below.
            {
                let mut right_sum = 0.0;
                let mut count = 0;

                for (data, prev_data) in sideways_wheel_data.iter().zip(&prev_sideways_wheel_data) {
                    if let Ok((travel, _)) = data
                        && let Ok((prev_travel, offset)) = prev_data
                    {
                        let delta_travel = travel - prev_travel;
                        count += 1;

                        right_sum += chord_scale * (delta_travel + offset * yaw);
                    }
                }

                if count != 0 {
                    local_displacement.x = right_sum / f64::from(count);
                }

                prev_sideways_wheel_data = sideways_wheel_data;
            }

            // MARK: Forward Wheels

            // Doing all the stuff with forward trackers in this block below.
            {
                let mut forward_sum = 0.0;
                let mut travel_sum = 0.0;

                let mut count = 0;
                let mut prev_count = 0;

                // Sum up all of our wheel values to determine average forward wheel travel and
                // average local forward-axis displacement.
                for (data, prev_data) in forward_wheel_data.iter().zip(&prev_forward_wheel_data) {
                    if let Ok((travel, _)) = data {
                        travel_sum += travel;
                        count += 1;

                        // For forward displacement, we need to calculate a delta of how much our
                        // wheel travel has changed. To do this, we need to have a record of both
                        // our current and previous wheel travel, meaning we can only consider
                        // wheels that returned `Ok(_)` in both the current AND previous loop
                        // iteration here.
                        if let Ok((prev_travel, offset)) = prev_data {
                            let delta_travel = travel - prev_travel;
                            prev_count += 1;

                            forward_sum += chord_scale * (delta_travel - offset * yaw);
                        }
                    }
                }

                // Average local forward displacement (change in forward positioning along the
                // robot's reference frame) using all functioning forward tracking wheels.
                if prev_count != 0 {
                    local_displacement.y = forward_sum / f64::from(prev_count);
                }

                // Calculate the forward wheel travel.
                //
                // This calculates the average of all forward tracking sensors on the robot as an
                // estimate of "how far we've driven". It's used for basic straight driving motion.
                if count != 0 {
                    data.forward_travel = travel_sum / f64::from(count);
                }

                prev_forward_wheel_data = forward_wheel_data;
            };

            // Used for estimating instantaneous velocity from wheel deltas.
            let dt = prev_time.elapsed();
            prev_time = Instant::now();

            // Linear/angular drivetrain velocity estimation
            //
            // TODO: Any kind of "dx/dt"-style differentiations here are flawed and will return zero
            //       sometimes due to the sample rate of our loop being higher than the sample rate
            //       of our sensor. We should also maybe consider EMA filtering this or something.
            data.linear_velocity = local_displacement.y / dt.as_secs_f64();

            data.angular_velocity = gyro
                .as_ref()
                .and_then(|gyro| gyro.angular_velocity().ok())
                .unwrap_or_else(|| delta_heading.as_radians() / dt.as_secs_f64());

            // Update global position by converting our local displacement vector into a global
            // offset (by rotating our local offset by our heading). Each iteration, we apply this
            // estimate of our change in position to get a new estimate of the global position.
            //
            // If all this seems like gibberish to you, check out <https://www.youtube.com/watch?v=ZW7T6EFyYnc>.
            data.position += Vec2::from_angle(avg_heading.as_radians()).rotate(local_displacement);
            data.ready = !heading_source_changed;
        }
    }

    // MARK: Setters
}
