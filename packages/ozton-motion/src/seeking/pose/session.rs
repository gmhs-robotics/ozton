use std::time::{Duration, Instant};

use ozton_diagnostics::{log_event, log_row};
use ozton_localization::{Feedback, LocalizationEstimate};
use ozton_math::{
    geometry::wrap_heading,
    uom::si::{f64::Velocity, velocity::meter_per_second},
};

use super::{MotionExecution, PathActions, PoseMotionError, RangeResetDrive};
use crate::basic::differential::{
    DifferentialFeedback, DifferentialTarget, DriveOptions, TerminalPose,
};

pub(super) struct Trajectory<'a> {
    pub path: &'a ozton_math::path::Path,
    pub reverse: bool,
    pub target: ozton_math::geometry::Pose,
    pub timeout: Duration,
}

/// A path segment's controller and nonblocking mechanism actions.
pub(super) struct PathStage<'s, 'a> {
    pub config: crate::MotionConfig,
    pub end_distance: f64,
    pub name: Option<&'a str>,
    pub actions: &'s mut PathActions<'a>,
}

/// A direction cusp must reach zero velocity, but does not require a dwell.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum Arrival {
    Stop,
    Reverse,
}

pub(super) struct PoseSession<'a, D: MotionExecution> {
    pub(super) drive: &'a mut D,
    pub(super) options: DriveOptions,
    config: crate::MotionConfig,
    previous_tick: Instant,
    started: Instant,
    finished: bool,
}

impl<'a, D: MotionExecution> PoseSession<'a, D> {
    pub(super) fn new(
        drive: &'a mut D,
        options: DriveOptions,
        config: crate::MotionConfig,
    ) -> Result<Self, PoseMotionError<D::Error>> {
        if !options.valid() || !config.valid() {
            return Err(PoseMotionError::Configuration);
        }
        // Start every independent command with stopped output.
        drive.brake().map_err(PoseMotionError::StopFailed)?;
        let (lkp, lki, lkd) = config.tuning.linear.gains();
        let (akp, aki, akd) = config.tuning.angular.gains();
        log_row!(
            Debug,
            "MOTION_CONFIG",
            "linear_kp={lkp},linear_ki={lki},linear_kd={lkd},angular_kp={akp},angular_ki={aki},angular_kd={akd},wheel_limit_mps={:.6},accel_mps2={:.6},decel_mps2={:.6},lateral_accel_mps2={:.6},track_width_m={:.6},brake_latency_ms={}",
            options.maximum_wheel_speed.value,
            config.limits.acceleration.value,
            config.limits.deceleration.value,
            config.limits.lateral_acceleration.value,
            options.track_width.value,
            config.limits.brake_latency.as_millis()
        );
        Ok(Self {
            drive,
            options,
            config,
            previous_tick: Instant::now(),
            started: Instant::now(),
            finished: false,
        })
    }

    async fn tick(&mut self) -> Result<Duration, PoseMotionError<D::Error>> {
        let period = self.config.timing.period;
        let deadline = self
            .previous_tick
            .checked_add(period)
            .ok_or(PoseMotionError::Configuration)?;
        self.drive.wait_until(deadline).await;
        let now = Instant::now();
        let dt = now.saturating_duration_since(self.previous_tick);
        self.previous_tick = now;
        ozton_diagnostics::profile::record(
            ozton_diagnostics::profile::Stage::ControlInterval,
            dt,
            period + Duration::from_millis(1),
        );
        if !self.drive.motion_active() {
            return Err(PoseMotionError::Cancelled);
        }
        if dt.is_zero() || dt > self.config.timing.maximum_gap {
            return Err(PoseMotionError::ControlGap);
        }
        Ok(dt)
    }

    pub(super) async fn run(
        &mut self,
        target: DifferentialTarget,
        timeout: Duration,
    ) -> Result<(), PoseMotionError<D::Error>> {
        self.run_with_output(target, timeout, |drive, wheels| drive.drive_wheels(wheels))
            .await
    }

    pub(super) async fn run_with_output(
        &mut self,
        target: DifferentialTarget,
        timeout: Duration,
        mut output: impl FnMut(&mut D, [Velocity; 2]) -> Result<(), D::Error>,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let status = match target {
            DifferentialTarget::Turn { .. } | DifferentialTarget::TurnToPoint { .. } => {
                "Autonomous: turn"
            }
            DifferentialTarget::Swing { .. } => "Autonomous: swing turn",
            DifferentialTarget::DriveToPoint { .. } => "Autonomous: drive to point",
            DifferentialTarget::DriveToPose { .. } => "Autonomous: Boomerang pose",
            DifferentialTarget::DriveDistance { .. } => "Autonomous: drive distance",
            DifferentialTarget::DriveArc { .. } => "Autonomous: drive arc",
        };
        log_event!(Info, "motion", "{status}: started");
        log_row!(
            Debug,
            "MOTION_COMMAND",
            "state=start,command={status},timeout_ms={}",
            timeout.as_millis()
        );
        let [x, y, heading, travel, radius, sweep] = match target {
            DifferentialTarget::Turn { heading } | DifferentialTarget::Swing { heading, .. } => [
                f64::NAN,
                f64::NAN,
                heading.value,
                f64::NAN,
                f64::NAN,
                f64::NAN,
            ],
            DifferentialTarget::TurnToPoint { target }
            | DifferentialTarget::DriveToPoint { target, .. } => {
                [target.x, target.y, f64::NAN, f64::NAN, f64::NAN, f64::NAN]
            }
            DifferentialTarget::DriveToPose { target, .. } => [
                target.position.x,
                target.position.y,
                target.heading.value,
                f64::NAN,
                f64::NAN,
                f64::NAN,
            ],
            DifferentialTarget::DriveDistance {
                meters, heading, ..
            } => [
                f64::NAN,
                f64::NAN,
                heading.map_or(f64::NAN, |h| h.value),
                meters.value,
                f64::NAN,
                f64::NAN,
            ],
            DifferentialTarget::DriveArc { radius, sweep, .. } => [
                f64::NAN,
                f64::NAN,
                f64::NAN,
                f64::NAN,
                radius.value,
                sweep.value,
            ],
        };
        log_row!(
            Debug,
            "MOTION_TARGET",
            "command={status},x_m={x:.5},y_m={y:.5},heading_rad={heading:.5},travel_m={travel:.5},radius_m={radius:.5},sweep_rad={sweep:.5}"
        );
        let result = self.run_stage(target, timeout, status, &mut output).await;
        match &result {
            Ok(()) => {
                log_row!(Debug, "MOTION_COMMAND", "state=complete,command={status}");
                log_event!(Info, "motion", "{status}: settled");
            }
            Err(error) => log_row!(
                Debug,
                "MOTION_COMMAND",
                "state=failed,command={status},reason={}",
                error.reason()
            ),
        }
        result
    }

    /// Run every autonomous controller on the same fresh-feedback, actuator
    /// and settling lifecycle. The controller supplies only its wheel command.
    async fn execute(
        &mut self,
        config: crate::MotionConfig,
        options: DriveOptions,
        timeout: Duration,
        mut step: impl FnMut(
            &mut DifferentialFeedback,
            &mut ControlTick<'_>,
        ) -> Result<ControlCommand, PoseMotionError<D::Error>>,
        output: &mut impl FnMut(&mut D, [Velocity; 2]) -> Result<(), D::Error>,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let started = Instant::now();
        self.previous_tick = started;
        let baseline = read_feedback(self.drive)
            .map_err(PoseMotionError::Tracking)?
            .map(|f| f.time);
        let mut controller = DifferentialFeedback::new(config.tuning).with_config(config);
        let mut settled = Duration::ZERO;
        let mut warnings = MotionWarnings::default();
        let mut last = None;
        let mut fresh_at = Instant::now();
        let mut interrupted = false;
        loop {
            let tick = self.tick().await?;
            let _span = ozton_diagnostics::profile::Span::new(
                ozton_diagnostics::profile::Stage::MotionTick,
            );
            warnings.stage(started.elapsed(), timeout);
            let Some(feedback) = read_feedback(self.drive).map_err(PoseMotionError::Tracking)?
            else {
                self.drive
                    .emergency_brake()
                    .map_err(PoseMotionError::StopFailed)?;
                controller.reset();
                settled = Duration::ZERO;
                interrupted = true;
                if fresh_at.elapsed() > config.timing.maximum_gap {
                    return Err(PoseMotionError::ControlGap);
                }
                continue;
            };
            let previous = last.or(baseline);
            if previous.is_some_and(|time| feedback.time <= time) {
                if fresh_at.elapsed() > config.timing.maximum_gap {
                    return Err(PoseMotionError::ControlGap);
                }
                continue;
            }
            let dt = last.map_or(tick, |time| feedback.time.saturating_sub(time));
            if dt > config.timing.maximum_gap {
                return Err(PoseMotionError::ControlGap);
            }
            if !valid_feedback(feedback) {
                return Err(PoseMotionError::InvalidFeedback);
            }
            last = Some(feedback.time);
            fresh_at = Instant::now();
            warnings.pose(self.drive.localization().pose_trusted());
            let command = step(
                &mut controller,
                &mut ControlTick {
                    feedback,
                    dt,
                    elapsed: started.elapsed(),
                    interrupted,
                    warnings: &mut warnings,
                    settled: &mut settled,
                    settle: options.settle,
                },
            )?;
            interrupted = false;
            controller.record_applied_wheels(command.wheels);
            if command.emergency_brake {
                self.drive
                    .emergency_brake()
                    .map_err(PoseMotionError::StopFailed)?;
            } else if command.complete || command.brake {
                self.drive.brake().map_err(PoseMotionError::StopFailed)?;
            } else {
                output(
                    self.drive,
                    command.wheels.map(Velocity::new::<meter_per_second>),
                )
                .map_err(PoseMotionError::Drive)?;
            }
            self.drive.log_step(self.started.elapsed(), dt);
            if command.complete {
                return Ok(());
            }
        }
    }

    async fn run_stage(
        &mut self,
        target: DifferentialTarget,
        timeout: Duration,
        status: &str,
        output: &mut impl FnMut(&mut D, [Velocity; 2]) -> Result<(), D::Error>,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let config = self.config;
        let options = self.options;
        let mut progress = Progress::default();
        let mut terminal = None;
        self.execute(
            config,
            options,
            timeout,
            |controller, tick| {
                let feedback = tick.feedback;
                let dt = tick.dt;
                if tick.interrupted {
                    progress = Progress::default();
                    terminal = None;
                }
                let mut braking = false;
                let (wheels, ready) = if let DifferentialTarget::DriveToPose {
                    target: pose,
                    reverse,
                    ..
                } = target
                    && (terminal.is_some()
                        || feedback.pose.position.distance(pose.position)
                            <= options.position_tolerance.value)
                {
                    let capture = terminal.get_or_insert_with(TerminalPose::default);
                    let (wheels, ready, _, changed) =
                        capture.step(controller, pose, reverse, feedback, dt, options);
                    braking = capture.is_braking();
                    if changed {
                        progress = Progress::default();
                    }
                    (wheels, ready)
                } else {
                    controller.step(target, feedback, dt, options)
                };
                let complete = tick.arrived(ready);
                if progress.stalled(target, feedback, ready, dt, config.timing.progress_timeout) {
                    tick.warnings.progress();
                } else {
                    tick.warnings.progress_reported = false;
                }
                log_row!(
                    Trace,
                    "MOTION_STEP",
                    "command={status},sample_us={},elapsed_ms={},dt_us={},x_m={:.5},y_m={:.5},heading_rad={:.5},forward_mps={:.5},lateral_mps={:.5},yaw_radps={:.5},travel_m={:.5},left_target_mps={:.5},right_target_mps={:.5},complete={complete}",
                    feedback.time.as_micros(),
                    tick.elapsed.as_millis(),
                    dt.as_micros(),
                    feedback.pose.position.x,
                    feedback.pose.position.y,
                    feedback.pose.heading.value,
                    feedback.forward_velocity.value,
                    feedback.lateral_velocity.value,
                    feedback.angular_velocity.value,
                    feedback.forward_travel.value,
                    wheels[0],
                    wheels[1]
                );
                Ok(ControlCommand {
                    wheels,
                    complete,
                    brake: terminal.is_some() && ready,
                    emergency_brake: braking,
                })
            },
            output,
        )
        .await
    }

    /// Execute a prepared trajectory with live feedback and a common PID terminal stage.
    pub(super) async fn follow(
        &mut self,
        trajectory: Trajectory<'_>,
        actions: &mut PathActions<'_>,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let mut stages = [PathStage {
            config: self.config,
            end_distance: trajectory.path.end().distance,
            name: None,
            actions,
        }];
        self.follow_group(trajectory, &mut stages, Arrival::Stop)
            .await
    }

    /// Execute a shared path profile, switching tracking laws at measured
    /// transit gates. The feedback loop and wheel limiter never restart there.
    pub(super) async fn follow_group(
        &mut self,
        trajectory: Trajectory<'_>,
        stages: &mut [PathStage<'_, '_>],
        arrival: Arrival,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let Trajectory {
            path,
            reverse,
            target,
            timeout,
        } = trajectory;
        use crate::pursuit::profiled::ProfiledFollower;
        let width = self.options.track_width;
        let configs: Vec<_> = stages
            .iter()
            .map(|stage| {
                stage
                    .config
                    .constrained_for_path(stage.config.options(width), path.limits())
            })
            .collect();
        let end_times: Vec<_> = stages
            .iter()
            .map(|stage| {
                path.at_distance(ozton_math::qty!((stage.end_distance) m))
                    .time
            })
            .collect();
        let mut followers = Vec::with_capacity(stages.len());
        for (index, &(config, _)) in configs.iter().enumerate() {
            if index > 0 && config == configs[index - 1].0 {
                followers.push(None);
                continue;
            }
            let follower =
                ProfiledFollower::new(path, config.path.lookahead, config.tuning.angular)
                    .with_config(config.path)
                    .prepare(config.timing.period, reverse)
                    .ok_or(PoseMotionError::Configuration)?;
            followers.push(Some(follower));
        }
        let mut follower = followers[0].take().ok_or(PoseMotionError::Configuration)?;
        let mut stage_index = 0;
        let (mut config, mut options) = configs[0];
        let initial_config = config;
        let final_options = configs.last().ok_or(PoseMotionError::Configuration)?.1;
        let mut stage_start = 0.0;
        let mut best = f64::INFINITY;
        let mut no_progress = Duration::ZERO;
        let mut terminal_pose = TerminalPose::default();
        let mut terminal = false;
        log_event!(Info, "motion", "Trajectory: started");
        let end = path.end();
        let planned_duration = follower.planned_duration();
        log_row!(
            Debug,
            "PATH_PLAN",
            "duration_s={:.5},length_m={:.5},reverse={reverse},x_m={:.5},y_m={:.5},heading_rad={:.5},samples={},lookahead_m={:.5},guidance={:?}",
            planned_duration,
            end.distance,
            target.position.x,
            target.position.y,
            target.heading.value,
            path.samples().len(),
            config.path.lookahead.value,
            config.path.guidance
        );
        let started = Instant::now();
        self.execute(
            initial_config,
            final_options,
            timeout,
            |controller, tick| {
                let feedback = tick.feedback;
                let dt = tick.dt;
                if tick.interrupted {
                    best = f64::INFINITY;
                    no_progress = Duration::ZERO;
                    terminal_pose = TerminalPose::default();
                    follower.reset_feedback();
                }
                let observation = follower.observe(
                    feedback.pose,
                    ozton_math::qty!((options.maximum_wheel_speed.value * dt.as_secs_f64() * 2.0) m),
                );
                while stage_index + 1 < stages.len()
                    && observation.distance >= stages[stage_index].end_distance - 1e-7
                {
                    let stage = &mut stages[stage_index];
                    stage.actions.advance(stage.end_distance - stage_start, 0.0, true)
                        .map_err(PoseMotionError::Mechanism)?;
                    let junction = path.at_distance(ozton_math::qty!((stage.end_distance) m)).pose;
                    log_checkpoint(stage.name, "blend", junction, reverse, feedback.pose, feedback.forward_velocity.value);
                    stage_start = stage.end_distance;
                    stage_index += 1;
                    (config, options) = configs[stage_index];
                    log_row!(Info, "MOTION_GUIDANCE", "index={stage_index},guidance={:?},reverse={reverse},progress_m={:.5}", config.path.guidance, observation.distance);
                    if let Some(mut next) = followers[stage_index].take() {
                        next.seek(observation.distance);
                        follower = next;
                        controller.reconfigure(config);
                    }
                }
                let cross = observation.cross_track;
                tick.warnings.cross_track(cross, config.path.maximum_cross_track_error.value);
                let stage = &mut stages[stage_index];
                let remaining = if stage_index + 1 == end_times.len() {
                    observation.remaining_time
                } else {
                    (end_times[stage_index] - observation.nominal_progress_time).max(0.0)
                        * follower.planned_duration() / path.end().time
                };
                stage.actions
                    .advance((observation.distance - stage_start).max(0.0), remaining, false)
                    .map_err(PoseMotionError::Mechanism)?;
                let nominal_progress_time = observation.nominal_progress_time;
                let distance = feedback.pose.position.distance(target.position);
                let heading_error = wrap_heading(target.heading - feedback.pose.heading)
                    .value
                    .abs();
                if !terminal && stage_index + 1 == stages.len() && follower.is_terminal(options.position_tolerance.value) {
                    terminal = true;
                    log_event!(Info, "motion", "Trajectory: entering final pose correction");
                    best = f64::INFINITY;
                    no_progress = Duration::ZERO;
                    controller.reset_feedback();
                }
                let cusp_ready = terminal && arrival == Arrival::Reverse
                    && distance <= options.position_tolerance.value
                    && heading_error <= options.heading_tolerance.value
                    && feedback.forward_velocity.value.powi(2) + feedback.lateral_velocity.value.powi(2)
                        <= options.speed_tolerance.value.powi(2)
                    && feedback.angular_velocity.value.abs() <= options.angular_speed_tolerance.value;
                let (requested, wheels, ready, objective, phase_changed) = if cusp_ready {
                    ([0.0; 2], [0.0; 2], true, 0.0, false)
                } else if terminal {
                    let (wheels, ready, objective, changed) =
                        terminal_pose.step(controller, target, reverse, feedback, dt, options);
                    (wheels, wheels, ready, objective, changed)
                } else {
                    let desired = follower.command(
                        feedback.pose,
                        reverse,
                        [feedback.lateral_velocity, feedback.forward_velocity],
                        feedback.angular_velocity,
                        config.limits.brake_latency,
                        dt,
                    );
                    let (wheels, ready) = controller.limit_step(desired, false, dt, options);
                    (
                        desired,
                        wheels,
                        ready,
                        end.distance - observation.distance,
                        false,
                    )
                };
                if phase_changed {
                    best = f64::INFINITY;
                    no_progress = Duration::ZERO;
                }
                if ready || objective < best - 0.002 {
                    best = objective;
                    no_progress = Duration::ZERO;
                    tick.warnings.progress_reported = false;
                } else {
                    no_progress = no_progress.saturating_add(dt);
                }
                if no_progress >= config.timing.progress_timeout {
                    tick.warnings.progress();
                }
                let complete = if arrival == Arrival::Reverse { ready } else { tick.arrived(ready) };
                log_row!(
                    Trace,
                    "PATH_TICK",
                    "sample_ms={},nominal_progress_s={nominal_progress_time:.5},progress_m={:.5},cross_m={cross:.5},x_m={:.5},y_m={:.5},heading_rad={:.5},error_m={distance:.5},heading_error_rad={heading_error:.5},left_mps={:.5},right_mps={:.5},terminal={terminal},complete={complete},dt_us={},requested_left_mps={:.5},requested_right_mps={:.5},forward_mps={:.5},lateral_mps={:.5},yaw_radps={:.5}",
                    feedback.time.as_millis(),
                    observation.distance,
                    feedback.pose.position.x,
                    feedback.pose.position.y,
                    feedback.pose.heading.value,
                    wheels[0],
                    wheels[1],
                    dt.as_micros(),
                    requested[0],
                    requested[1],
                    feedback.forward_velocity.value,
                    feedback.lateral_velocity.value,
                    feedback.angular_velocity.value
                );
                Ok(ControlCommand {
                    wheels,
                    complete,
                    brake: terminal && ready,
                    emergency_brake: terminal && terminal_pose.is_braking(),
                })
            },
            &mut |drive, wheels| drive.drive_wheels(wheels),
        )
        .await?;
        stages[stage_index]
            .actions
            .advance(end.distance - stage_start, 0.0, true)
            .map_err(PoseMotionError::Mechanism)?;
        let elapsed = started.elapsed();
        let pose = self.drive.localization().pose();
        let error = pose.position.distance(target.position);
        let heading_error = wrap_heading(target.heading - pose.heading).value.abs();
        let passed = error <= options.position_tolerance.value
            && heading_error <= options.heading_tolerance.value;
        log_row!(
            Info,
            "PATH_RESULT",
            "elapsed_us={},target_x_m={:.6},target_y_m={:.6},target_heading_rad={:.6},actual_x_m={:.6},actual_y_m={:.6},actual_heading_rad={:.6},error_m={error:.6},heading_error_rad={heading_error:.6},passed={passed}",
            elapsed.as_micros(),
            target.position.x,
            target.position.y,
            target.heading.value,
            pose.position.x,
            pose.position.y,
            pose.heading.value
        );
        log_row!(
            Info,
            "PATH_TIMING",
            "planned_s={:.5},actual_s={:.5},ratio={:.5}",
            planned_duration,
            elapsed.as_secs_f64(),
            elapsed.as_secs_f64() / planned_duration
        );
        let final_name = stages[stage_index].name;
        let forward = read_feedback(self.drive)
            .ok()
            .flatten()
            .map_or(f64::NAN, |feedback| feedback.forward_velocity.value);
        log_checkpoint(
            final_name,
            if arrival == Arrival::Reverse {
                "reverse"
            } else {
                "stop"
            },
            end.pose,
            reverse,
            pose,
            forward,
        );
        log_event!(Info, "motion", "Trajectory: settled");
        Ok(())
    }

    pub(super) async fn try_distance_reset(
        &mut self,
        timeout: Duration,
    ) -> Result<bool, PoseMotionError<D::Error>>
    where
        D: RangeResetDrive,
    {
        self.drive.brake().map_err(PoseMotionError::StopFailed)?;
        let started = Instant::now();
        self.previous_tick = started;
        self.drive.localization_mut().request_distance_reset();
        log_event!(
            Info,
            "localization",
            "Stationary distance correction: requested"
        );
        loop {
            let dt = self.tick().await?;
            if started.elapsed() >= timeout {
                log_event!(
                    Warn,
                    "localization",
                    "Stationary distance correction: no accepted fix before timeout"
                );
                return Ok(false);
            }
            let feedback = read_feedback(self.drive).map_err(PoseMotionError::Tracking)?;
            self.drive.log_step(self.started.elapsed(), dt);
            if let Some(feedback) = feedback {
                if !valid_feedback(feedback) {
                    return Err(PoseMotionError::InvalidFeedback);
                }
                if feedback.reset_accepted {
                    log_event!(
                        Info,
                        "localization",
                        "Stationary distance correction: accepted"
                    );
                    return Ok(true);
                }
            }
        }
    }

    pub(super) fn finish<T>(
        mut self,
        outcome: Result<T, PoseMotionError<D::Error>>,
    ) -> Result<T, PoseMotionError<D::Error>> {
        self.drive.localization_mut().cancel_distance_reset();
        let stop = if outcome.is_ok() {
            self.drive.brake()
        } else {
            self.drive.emergency_brake()
        };
        let result = match (outcome, stop) {
            (outcome, Ok(())) => outcome,
            (Ok(_), Err(stop)) => Err(PoseMotionError::StopFailed(stop)),
            (Err(cause), Err(stop)) => Err(PoseMotionError::StopFailedAfter {
                cause: Box::new(cause),
                stop,
            }),
        };
        self.finished = true;
        if let Err(error) = &result {
            log_event!(Warn, "motion", "Motion ended: {}", error.reason());
        }
        result
    }
}

fn log_checkpoint(
    name: Option<&str>,
    junction: &str,
    target: ozton_math::geometry::Pose,
    reverse: bool,
    actual: ozton_math::geometry::Pose,
    speed: f64,
) {
    let name = name.unwrap_or("path");
    let heading = target.heading + ozton_math::qty!((if reverse { 180.0 } else { 0.0 }) deg);
    let error = target.position.distance(actual.position);
    let heading_error = wrap_heading(heading - actual.heading).value;
    log_row!(
        Info,
        "MOTION_CHECKPOINT",
        "stage={name},junction={junction},target_x_m={:.6},target_y_m={:.6},target_heading_rad={:.6},x_m={:.6},y_m={:.6},heading_rad={:.6},error_m={error:.6},heading_error_rad={heading_error:.6},forward_mps={speed:.5}",
        target.position.x,
        target.position.y,
        heading.value,
        actual.position.x,
        actual.position.y,
        actual.heading.value
    );
    let _ = (name, junction, error, heading_error, speed);
}

/// Fresh feedback and arrival state shared by every autonomous controller.
struct ControlTick<'a> {
    feedback: Feedback,
    dt: Duration,
    elapsed: Duration,
    interrupted: bool,
    warnings: &'a mut MotionWarnings,
    settled: &'a mut Duration,
    settle: Duration,
}

impl ControlTick<'_> {
    fn arrived(&mut self, ready: bool) -> bool {
        *self.settled = if ready {
            self.settled.saturating_add(self.dt)
        } else {
            Duration::ZERO
        };
        ready && *self.settled >= self.settle
    }
}

struct ControlCommand {
    wheels: [f64; 2],
    complete: bool,
    brake: bool,
    emergency_brake: bool,
}

impl<D: MotionExecution> Drop for PoseSession<'_, D> {
    fn drop(&mut self) {
        if !self.finished {
            self.drive.localization_mut().cancel_distance_reset();
            let stopped = self.drive.emergency_brake();
            if stopped.is_ok() {
                log_event!(Info, "motion", "Motion interrupted; brake requested");
            } else {
                log_event!(Error, "motion", "Motion interrupted; brake request failed");
            }
        }
    }
}

fn read_feedback<D: MotionExecution>(
    drive: &D,
) -> Result<Option<Feedback>, ozton_localization::Fault> {
    drive.localization().feedback()
}

fn valid_feedback(feedback: Feedback) -> bool {
    feedback.pose.is_finite()
        && [
            feedback.forward_travel.value,
            feedback.forward_velocity.value,
            feedback.lateral_velocity.value,
            feedback.angular_velocity.value,
        ]
        .into_iter()
        .all(f64::is_finite)
}

#[derive(Default)]
struct Progress {
    start: Option<Feedback>,
    best: Option<f64>,
    elapsed: Duration,
}

impl Progress {
    fn stalled(
        &mut self,
        target: DifferentialTarget,
        feedback: Feedback,
        ready: bool,
        dt: Duration,
        timeout: Duration,
    ) -> bool {
        let start = *self.start.get_or_insert(feedback);
        let pose = feedback.pose;
        let error = match target {
            DifferentialTarget::DriveToPoint { target, .. } => pose.position.distance(target),
            DifferentialTarget::DriveToPose { target, .. } => {
                pose.position.distance(target.position)
                    + 0.1 * wrap_heading(target.heading - pose.heading).value.abs()
            }
            DifferentialTarget::Turn { heading } | DifferentialTarget::Swing { heading, .. } => {
                wrap_heading(heading - pose.heading).value.abs()
            }
            DifferentialTarget::TurnToPoint { target } => {
                let delta = target - pose.position;
                if delta.length() <= 1e-9 {
                    0.0
                } else {
                    wrap_heading(ozton_math::geometry::bearing(delta) - pose.heading)
                        .value
                        .abs()
                }
            }
            DifferentialTarget::DriveDistance { meters, .. } => {
                (meters.value - (feedback.forward_travel.value - start.forward_travel.value)).abs()
            }
            DifferentialTarget::DriveArc { radius, sweep, .. } => {
                let remaining = (radius.value * sweep.value.abs()
                    - (feedback.forward_travel.value - start.forward_travel.value))
                    .abs();
                remaining
                    + radius.value
                        * wrap_heading(start.pose.heading + sweep - pose.heading)
                            .value
                            .abs()
            }
        };
        let improvement = match target {
            DifferentialTarget::Turn { .. }
            | DifferentialTarget::Swing { .. }
            | DifferentialTarget::TurnToPoint { .. } => 0.01,
            _ => 0.002,
        };
        let best = self.best.get_or_insert(error);
        if ready || error < *best - improvement {
            *best = error;
            self.elapsed = Duration::ZERO;
        } else {
            self.elapsed += dt;
        }
        self.elapsed >= timeout
    }
}

// Monitoring thresholds never change wheel output or terminate a valid motion.
#[derive(Default)]
struct MotionWarnings {
    stage_reported: bool,
    progress_reported: bool,
    cross_track_reported: bool,
    pose_reported: bool,
}

impl MotionWarnings {
    fn stage(&mut self, elapsed: Duration, threshold: Duration) {
        if elapsed >= threshold && !self.stage_reported {
            self.stage_reported = true;
            log_event!(
                Warn,
                "motion",
                "Stage exceeded expected duration; continuing feedback"
            );
        }
    }

    fn progress(&mut self) {
        if !self.progress_reported {
            self.progress_reported = true;
            log_event!(Warn, "motion", "Low progress; continuing feedback");
        }
    }

    fn cross_track(&mut self, error: f64, threshold: f64) {
        if error > threshold && !self.cross_track_reported {
            log_event!(
                Warn,
                "motion",
                "Cross-track error above tuning threshold; continuing feedback"
            );
            self.cross_track_reported = true;
        } else if error <= threshold * 0.5 {
            self.cross_track_reported = false;
        }
    }

    fn pose(&mut self, trusted: bool) {
        if !trusted && !self.pose_reported {
            log_event!(
                Warn,
                "motion",
                "Low pose confidence; continuing with finite feedback"
            );
        }
        self.pose_reported = !trusted;
    }
}
