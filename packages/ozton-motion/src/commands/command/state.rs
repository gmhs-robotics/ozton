//! Follower and settling state for a running command.
use ozton_runtime::log_row;

use super::{pid::MotionPid, *};

#[derive(Default)]
pub(in crate::commands) struct State<'path> {
    pid: MotionPid,
    progress: Option<Progress<'path>>,
    settled: Duration,
    fresh_range: Option<Duration>,
    range_elapsed: Duration,
    settled_range: Option<Duration>,
    start_travel: Option<f64>,
    start_heading: Option<Angle>,
    next_marker: usize,
}

pub(in crate::commands) struct Step {
    pub wheels: [f64; 2],
    pub complete: bool,
    pub progress: f64,
    pub lost: bool,
}

impl<'path> State<'path> {
    pub(in crate::commands) fn new(tuning: crate::commands::DriveTuning) -> Self {
        Self {
            pid: MotionPid::new(tuning),
            ..Self::default()
        }
    }

    pub(in crate::commands) fn pause(&mut self) {
        self.pid.reset();
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
        let error_at = |heading| wrap_heading(heading - feedback.pose.heading).value;
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
        let forward_rate = feedback.forward_velocity.value;
        let yaw_rate = feedback.angular_velocity.value;
        let yaw_limit = 2.0 * options.maximum_wheel_speed.value / options.track_width.value;
        let angular = |pid: &mut MotionPid, error| pid.yaw(error, yaw_rate, 0.0, yaw_limit, dt);
        let mut contact_motion = |touched, heading: Angle, speed, limit, settle: Duration| {
            let error = error_at(heading);
            if touched {
                self.pid.reset();
                (
                    [0.0; 2],
                    slow && error.abs() <= options.heading_tolerance.value,
                    settle.max(Duration::from_millis(30)),
                )
            } else {
                let linear = if error.abs() < 0.2 { speed } else { 0.0 };
                (
                    desaturate(drive(linear, angular(&mut self.pid, error)), limit),
                    false,
                    settle,
                )
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
                let reference = path
                    .at_distance(Length::new::<meter>(progress.distance()))
                    .pose;
                let tracking_error = reference.position.distance(feedback.pose.position);
                lost = tracking_error > 0.35
                    || (matches!(follower, Follower::Ramsete { .. })
                        && elapsed.as_secs_f64() >= end.time
                        && progress.distance() < end.distance - 0.20);
                let heading = end.pose.heading
                    + Angle::new::<radian>(if *reverse { core::f64::consts::PI } else { 0.0 });
                let ready = progress.distance() >= end.distance - options.position_tolerance.value
                    && feedback.pose.position.distance(end.pose.position)
                        <= options.position_tolerance.value
                    && error_at(heading).abs() <= options.heading_tolerance.value;
                let sample = FollowerSample {
                    pose: feedback.pose,
                    elapsed: Time::new::<second>(elapsed.as_secs_f64()),
                    period: Time::new::<second>(dt.as_secs_f64()),
                };
                let wheels = if ready && slow {
                    [0.0; 2]
                } else if progress.is_terminal(*follower, sample, options.position_tolerance.value)
                {
                    let delta = end.pose.position - feedback.pose.position;
                    let distance = delta.length();
                    let (linear, yaw_error) = if distance > options.position_tolerance.value {
                        let toward = ozton_math::geometry::bearing(delta);
                        let backwards = error_at(toward).abs() > core::f64::consts::FRAC_PI_2;
                        let heading = toward
                            + Angle::new::<radian>(if backwards {
                                core::f64::consts::PI
                            } else {
                                0.0
                            });
                        let error = error_at(heading);
                        let limit = path.limits().linear_speed.value * 0.4 * error.cos().max(0.0);
                        (
                            self.pid.forward(
                                if backwards { -distance } else { distance },
                                forward_rate,
                                limit,
                                dt,
                            ),
                            error,
                        )
                    } else {
                        (
                            self.pid.forward(0.0, forward_rate, 0.0, dt),
                            error_at(heading),
                        )
                    };
                    desaturate(
                        drive(linear, angular(&mut self.pid, yaw_error)),
                        path.limits().wheel_speed.value,
                    )
                } else {
                    follower::command(
                        *follower,
                        progress,
                        sample,
                        *reverse,
                        options.position_tolerance,
                    )
                };
                log_row!(
                    Trace,
                    "PATH_FOLLOWER",
                    "progress_m={:.5},end_m={:.5},reference_x_m={:.5},reference_y_m={:.5},cross_track_m={tracking_error:.5},end_error_m={:.5},end_heading_error_rad={:.5},profile_ms={:.0},reverse={reverse},ready={ready},lost={lost}",
                    progress.distance(),
                    end.distance,
                    reference.position.x,
                    reference.position.y,
                    feedback.pose.position.distance(end.pose.position),
                    error_at(heading),
                    end.time * 1000.0
                );
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
                let error = error_at(*heading);
                let ready = error.abs() <= options.heading_tolerance.value;
                (
                    drive(
                        0.0,
                        if ready && slow {
                            0.0
                        } else {
                            angular(&mut self.pid, error)
                        },
                    ),
                    ready && slow,
                    options.settle,
                )
            }
            Command::TurnToPoint { target } => {
                let delta = *target - feedback.pose.position;
                let error = if delta.length() <= options.position_tolerance.value {
                    0.0
                } else {
                    error_at(ozton_math::geometry::bearing(delta))
                };
                let ready = error.abs() <= options.heading_tolerance.value;
                (
                    drive(
                        0.0,
                        if ready && slow {
                            0.0
                        } else {
                            angular(&mut self.pid, error)
                        },
                    ),
                    ready && slow,
                    options.settle,
                )
            }
            Command::DriveToPoint {
                target,
                reverse,
                maximum_speed,
            } => {
                let delta = *target - feedback.pose.position;
                let distance = delta.length();
                let reached = distance <= options.position_tolerance.value;
                let heading = error_at(
                    ozton_math::geometry::bearing(delta)
                        + Angle::new::<radian>(if *reverse { core::f64::consts::PI } else { 0.0 }),
                );
                let sign = if *reverse { -1.0 } else { 1.0 };
                let limit = if reached || heading.abs() > 0.9 {
                    0.0
                } else {
                    maximum_speed.value * heading.cos()
                };
                let linear = self.pid.forward(distance * sign, forward_rate, limit, dt);
                let turn = if reached {
                    0.0
                } else {
                    angular(&mut self.pid, heading)
                };
                (drive(linear, turn), reached && slow, options.settle)
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
                let heading =
                    error_at(heading.unwrap_or_else(|| {
                        *self.start_heading.get_or_insert(feedback.pose.heading)
                    }));
                let reached = error.abs() <= options.position_tolerance.value;
                let facing = heading.abs() <= options.heading_tolerance.value;
                let limit = if reached || heading.abs() > 0.35 {
                    0.0
                } else {
                    maximum_speed.value
                };
                let speed = self.pid.forward(error, forward_rate, limit, dt);
                let turning = if facing && reached && slow {
                    0.0
                } else {
                    angular(&mut self.pid, heading)
                };
                (
                    drive(speed, turning),
                    reached && facing && slow,
                    options.settle,
                )
            }
            Command::DriveArc {
                radius,
                sweep,
                maximum_speed,
            } => {
                let start = *self
                    .start_travel
                    .get_or_insert(feedback.forward_travel.value);
                let arc_length = radius.value * sweep.value.abs();
                let travel = feedback.forward_travel.value - start;
                let remaining = arc_length - travel;
                let reached = remaining.abs() <= options.position_tolerance.value;
                let start_heading = *self.start_heading.get_or_insert(feedback.pose.heading);
                let curvature = sweep.value.signum() / radius.value;
                let heading = error_at(
                    start_heading
                        + if reached {
                            *sweep
                        } else {
                            Angle::new::<radian>(travel.clamp(0.0, arc_length) * curvature)
                        },
                );
                let facing = heading.abs() <= options.heading_tolerance.value;
                let wheel_limit = options.maximum_wheel_speed.value
                    / (1.0 + options.track_width.value * curvature.abs() * 0.5);
                let limit = if reached {
                    0.0
                } else {
                    maximum_speed.value.min(wheel_limit)
                };
                let speed = self.pid.forward(remaining, forward_rate, limit, dt);
                let feedforward = speed * curvature;
                let turning = if reached && facing && slow {
                    0.0
                } else {
                    let correction_limit = if reached { 0.5 } else { 0.35 };
                    self.pid
                        .yaw(heading, yaw_rate, feedforward, correction_limit, dt)
                };
                (
                    drive(speed, turning),
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
                    let heading_error = error_at(alignment.heading);
                    let position_ready = distance_error.abs() <= options.position_tolerance.value;
                    let heading_ready = heading_error.abs() <= options.heading_tolerance.value;
                    let limit = if position_ready || heading_error.abs() > 0.2 {
                        0.0
                    } else {
                        alignment.maximum_speed.value
                    };
                    let linear = self.pid.forward(
                        distance_error * alignment.facing.direction(),
                        forward_rate,
                        limit,
                        dt,
                    );
                    let turn = if heading_ready && position_ready && slow {
                        0.0
                    } else {
                        angular(&mut self.pid, heading_error)
                    };
                    (
                        drive(linear, turn),
                        position_ready && heading_ready && slow,
                        options.settle,
                    )
                } else {
                    self.pid.reset();
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
                *heading,
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

#[cfg(test)]
mod corner;
