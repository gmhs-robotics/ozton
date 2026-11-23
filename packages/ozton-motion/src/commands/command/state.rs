//! Follower and settling state for a running command.
use super::*;

#[derive(Default)]
pub(in crate::commands) struct State<'path> {
    progress: Option<Progress<'path>>,
    settled: Duration,
    fresh_range: Option<Duration>,
    range_elapsed: Duration,
    settled_range: Option<Duration>,
    start_travel: Option<f64>,
    start_heading: Option<f64>,
    next_marker: usize,
}

pub(in crate::commands) struct Step {
    pub wheels: [f64; 2],
    pub complete: bool,
    pub progress: f64,
    pub lost: bool,
}

impl<'path> State<'path> {
    pub(in crate::commands) fn pause(&mut self) {
        self.settled = Duration::ZERO;
        self.settled_range = None;
        self.range_elapsed = Duration::MAX;
    }

    pub(in crate::commands) fn step<R>(
        &mut self,
        command: &mut Command<'path, R>,
        robot: &mut R,
        feedback: Feedback,
        elapsed: Duration,
        dt: Duration,
        options: DriveOptions,
    ) -> Step {
        let slow = feedback
            .forward_velocity
            .value
            .hypot(feedback.lateral_velocity.value)
            <= options.speed_tolerance.value
            && feedback.angular_velocity.value.abs() <= options.angular_speed_tolerance.value;
        let error_at =
            |heading| wrap_heading(Angle::new::<radian>(heading - feedback.pose.heading));
        let drive = |linear, angular| {
            desaturate(
                wheel_speeds(
                    Velocity::new::<meter_per_second>(linear),
                    AngularVelocity::new::<radian_per_second>(angular),
                    options.track_width,
                )
                .map(|speed| speed.value),
                options.maximum_wheel_speed.value,
            )
        };
        let contact_motion = |touched, heading: f64, speed, limit, settle: Duration| {
            let error = error_at(heading);
            if touched {
                (
                    [0.0; 2],
                    slow && error.abs() <= options.heading_tolerance.value,
                    settle.max(Duration::from_millis(30)),
                )
            } else {
                let linear = if error.abs() < 0.2 { speed } else { 0.0 };
                (desaturate(drive(linear, 3.0 * error), limit), false, settle)
            }
        };
        let mut fresh_alignment_range = false;
        let mut lost = false;
        let (wheels, ready, settle) = match command {
            Command::Follow {
                path,
                follower,
                reverse,
                ..
            }
            | Command::FollowMarked {
                path,
                follower,
                reverse,
                ..
            } => {
                let progress = self.progress.get_or_insert_with(|| Progress::new(path));
                progress.update(
                    feedback.pose.position,
                    Length::new::<meter>(
                        options.maximum_wheel_speed.value * dt.as_secs_f64() * 2.0,
                    ),
                );
                let end = path.end();
                let tracking_error = path
                    .at_distance(Length::new::<meter>(progress.distance()))
                    .position
                    .distance(feedback.pose.position);
                lost = tracking_error > 0.35
                    || (matches!(follower, Follower::Ramsete { .. })
                        && elapsed.as_secs_f64() >= end.time
                        && progress.distance() < end.distance - 0.20);
                let heading = end.heading + if *reverse { core::f64::consts::PI } else { 0.0 };
                let ready = progress.distance() >= end.distance - options.position_tolerance.value
                    && feedback.pose.position.distance(end.position)
                        <= options.position_tolerance.value
                    && error_at(heading).abs() <= options.heading_tolerance.value;
                let wheels = if ready {
                    [0.0; 2]
                } else {
                    follower::command(
                        *follower,
                        progress,
                        FollowerSample {
                            pose: feedback.pose,
                            elapsed: Time::new::<second>(elapsed.as_secs_f64()),
                            period: Time::new::<second>(dt.as_secs_f64()),
                        },
                        *reverse,
                        options.position_tolerance,
                    )
                };
                if !lost {
                    let distance = if ready {
                        end.distance
                    } else {
                        progress.distance()
                    };
                    self.trigger(command, robot, distance);
                }
                (wheels, ready && slow, options.settle)
            }
            Command::Turn { heading } => {
                let error = error_at(heading.value);
                let ready = error.abs() <= options.heading_tolerance.value;
                (
                    drive(0.0, if ready { 0.0 } else { 3.0 * error }),
                    ready && slow,
                    options.settle,
                )
            }
            Command::TurnToPoint { target } => {
                let delta =
                    glam::DVec2::new(target[0].value, target[1].value) - feedback.pose.position;
                let error = if delta.length() <= options.position_tolerance.value {
                    0.0
                } else {
                    error_at(ozton_math::geometry::bearing(delta))
                };
                let ready = error.abs() <= options.heading_tolerance.value;
                (
                    drive(0.0, if ready { 0.0 } else { 3.0 * error }),
                    ready && slow,
                    options.settle,
                )
            }
            Command::DriveToPoint {
                target,
                reverse,
                maximum_speed,
            } => {
                let delta = ozton_math::geometry::Pose::new(
                    target[0],
                    target[1],
                    Angle::new::<radian>(0.0),
                )
                .position
                    - feedback.pose.position;
                let distance = delta.length();
                let reached = distance <= options.position_tolerance.value;
                let heading = error_at(
                    ozton_math::geometry::bearing(delta)
                        + if *reverse { core::f64::consts::PI } else { 0.0 },
                );
                let linear = if reached || heading.abs() > 0.7 {
                    0.0
                } else {
                    (2.5 * distance).min(maximum_speed.value) * if *reverse { -1.0 } else { 1.0 }
                };
                (
                    drive(linear, if reached { 0.0 } else { 3.0 * heading }),
                    reached && slow,
                    options.settle,
                )
            }
            Command::DriveDistance {
                meters,
                heading,
                maximum_speed,
            } => {
                let start = *self
                    .start_travel
                    .get_or_insert(feedback.forward_travel.value);
                let error = meters.value - (feedback.forward_travel.value - start);
                let heading = error_at(heading.map_or_else(
                    || *self.start_heading.get_or_insert(feedback.pose.heading),
                    |angle| angle.value,
                ));
                let reached = error.abs() <= options.position_tolerance.value;
                let facing = heading.abs() <= options.heading_tolerance.value;
                let speed = if reached || heading.abs() > 0.35 {
                    0.0
                } else {
                    (2.5 * error).clamp(-maximum_speed.value, maximum_speed.value)
                };
                (
                    drive(speed, if facing { 0.0 } else { 3.0 * heading }),
                    reached && facing && slow,
                    options.settle,
                )
            }
            Command::Align(alignment) => {
                self.range_elapsed = self.range_elapsed.saturating_add(dt);
                let range = feedback.ranges[alignment.sensor].filter(|range| {
                    feedback
                        .time
                        .checked_sub(range.timestamp)
                        .is_some_and(|age| age <= Duration::from_millis(100))
                        && range.distance.value.is_finite()
                        && range.distance.value > 0.0
                        && self
                            .fresh_range
                            .is_none_or(|timestamp| range.timestamp >= timestamp)
                });
                if let Some(range) = range
                    && (self.fresh_range != Some(range.timestamp)
                        || self.range_elapsed <= Duration::from_millis(100))
                {
                    fresh_alignment_range = self.fresh_range != Some(range.timestamp);
                    if fresh_alignment_range {
                        self.fresh_range = Some(range.timestamp);
                        self.range_elapsed = Duration::ZERO;
                    }
                    let distance_error = range.distance.value - alignment.distance.value;
                    let heading_error = error_at(alignment.heading.value);
                    let position_ready = distance_error.abs() <= options.position_tolerance.value;
                    let heading_ready = heading_error.abs() <= options.heading_tolerance.value;
                    let linear = if position_ready || heading_error.abs() > 0.2 {
                        0.0
                    } else {
                        (2.0 * distance_error * alignment.facing.direction()).clamp(
                            -alignment.maximum_speed.value,
                            alignment.maximum_speed.value,
                        )
                    };
                    (
                        drive(
                            linear,
                            if heading_ready {
                                0.0
                            } else {
                                3.0 * heading_error
                            },
                        ),
                        position_ready && heading_ready && slow,
                        options.settle,
                    )
                } else {
                    ([0.0; 2], false, options.settle)
                }
            }
            Command::Contact(contact) => contact_motion(
                (contact.touched)(robot),
                contact.known_pose.heading,
                contact.speed.value,
                options.maximum_wheel_speed.value,
                options.settle,
            ),
            Command::ApproachContact {
                heading,
                speed,
                settle,
                touched,
            } => contact_motion(
                touched(robot, feedback, elapsed),
                heading.value,
                speed.value,
                speed.value.abs(),
                *settle,
            ),
            Command::DistanceReset => ([0.0; 2], feedback.reset_accepted, Duration::ZERO),
            Command::Wait(duration) => ([0.0; 2], elapsed >= *duration, Duration::ZERO),
            Command::WaitUntil(ready) => ([0.0; 2], ready(robot) && slow, options.settle),
            Command::WaitFor { ready, stable } => ([0.0; 2], ready(robot) && slow, *stable),
        };
        self.settled = if ready {
            self.settled.saturating_add(dt)
        } else {
            Duration::ZERO
        };
        let complete = if matches!(command, Command::Align(_)) {
            if !ready {
                self.settled_range = None;
                false
            } else {
                if fresh_alignment_range && self.settled_range.is_none() {
                    self.settled_range = Some(elapsed);
                }
                fresh_alignment_range
                    && self
                        .settled_range
                        .is_some_and(|start| elapsed.saturating_sub(start) >= settle)
            }
        } else {
            ready && self.settled >= settle
        };
        Step {
            wheels,
            complete,
            progress: self.progress.as_ref().map_or(0.0, Progress::distance),
            lost,
        }
    }

    fn trigger<R>(&mut self, command: &mut Command<'_, R>, robot: &mut R, distance: f64) {
        match command {
            Command::Follow { trigger, .. } => {
                if trigger
                    .as_ref()
                    .is_some_and(|marker| distance >= marker.distance.value)
                    && let Some(marker) = trigger.take()
                {
                    (marker.action)(robot);
                }
            }
            Command::FollowMarked { markers, .. } => {
                while let Some(marker) = markers.get(self.next_marker) {
                    if distance < marker.distance.value {
                        break;
                    }
                    self.next_marker += 1;
                    (marker.action)(robot);
                }
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod settling_tests {
    use super::*;
    #[test]
    fn captured_load_condition_requires_continuous_resistance_and_rest() {
        let threshold = 0.3;
        let mut ready = |torque: &f64| torque.is_finite() && *torque >= threshold;
        let mut command = Command::WaitFor {
            ready: &mut ready,
            stable: Duration::from_millis(30),
        };
        let options = DriveOptions::differential(
            Length::new::<meter>(0.3),
            Velocity::new::<meter_per_second>(1.0),
        );
        let mut state = State::default();
        let mut feedback = Feedback {
            time: Duration::ZERO,
            pose: Pose::default(),
            forward_travel: Length::new::<meter>(0.0),
            forward_velocity: Velocity::new::<meter_per_second>(0.0),
            lateral_velocity: Velocity::new::<meter_per_second>(0.0),
            angular_velocity: AngularVelocity::new::<radian_per_second>(0.0),
            ranges: [None; MAX_RANGES],
            reset_accepted: false,
        };
        let tick = Duration::from_millis(10);
        // Startup spikes and missing/low feedback do not complete a load wait.
        for mut sample in [0.4, 0.4, f64::NAN, 0.4, 0.1] {
            let step = state.step(&mut command, &mut sample, feedback, tick, tick, options);
            assert!(!step.complete);
            assert_eq!(step.wheels, [0.0; 2]);
        }
        feedback.angular_velocity = AngularVelocity::new::<radian_per_second>(0.5);
        for _ in 0..5 {
            assert!(
                !state
                    .step(&mut command, &mut 0.4, feedback, tick, tick, options)
                    .complete
            );
        }
        feedback.angular_velocity = AngularVelocity::new::<radian_per_second>(0.0);
        for expected in [false, false, true] {
            assert_eq!(
                state
                    .step(&mut command, &mut 0.4, feedback, tick, tick, options)
                    .complete,
                expected
            );
        }
    }
    #[test]
    fn sideways_motion_prevents_arrival_even_with_correct_position_and_heading() {
        let mut state = State::default();
        let mut command = Command::WaitUntil(|_: &()| true);
        let options = DriveOptions::differential(
            Length::new::<meter>(0.3),
            Velocity::new::<meter_per_second>(1.0),
        );
        let mut feedback = Feedback {
            time: Duration::ZERO,
            pose: Pose::default(),
            forward_travel: Length::new::<meter>(0.0),
            forward_velocity: Velocity::new::<meter_per_second>(0.0),
            lateral_velocity: Velocity::new::<meter_per_second>(0.2),
            angular_velocity: AngularVelocity::new::<radian_per_second>(0.0),
            ranges: [None; MAX_RANGES],
            reset_accepted: false,
        };
        let tick = Duration::from_millis(10);
        for _ in 0..30 {
            assert!(
                !state
                    .step(&mut command, &mut (), feedback, tick, tick, options)
                    .complete
            );
        }
        feedback.lateral_velocity = Velocity::new::<meter_per_second>(0.0);
        let mut completed = false;
        for _ in 0..12 {
            completed = state
                .step(&mut command, &mut (), feedback, tick, tick, options)
                .complete;
        }
        assert!(completed);
    }
}
