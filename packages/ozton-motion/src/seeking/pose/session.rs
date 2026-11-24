use std::time::{Duration, Instant};

use ozton_localization::Feedback;
use ozton_math::{
    geometry::{Pose, wrap_heading},
    uom::si::{f64::Velocity, velocity::meter_per_second},
};
use ozton_runtime::{
    clock::{CONTROL_PERIOD, CommandClock},
    log_row,
};
use vexide::competition::{self, CompetitionMode};

use super::{PoseDrive, PoseMotionError};
use crate::basic::differential::{
    DifferentialFeedback, DifferentialTarget, DriveOptions, DriveTuning,
};

pub(super) const MAX_CONTROL_GAP: Duration = Duration::from_millis(100);
const STALL_TIME: Duration = Duration::from_millis(800);

pub(super) struct PoseSession<'a, D: PoseDrive> {
    pub(super) drive: &'a mut D,
    pub(super) options: DriveOptions,
    tuning: DriveTuning,
    clock: CommandClock,
    started: Instant,
    budget: Duration,
    finished: bool,
}

impl<'a, D: PoseDrive> PoseSession<'a, D> {
    pub(super) fn new(
        drive: &'a mut D,
        options: DriveOptions,
        tuning: DriveTuning,
        budget: Duration,
    ) -> Result<Self, PoseMotionError<D::Error>> {
        if !options.valid() || !tuning.valid() || budget < CONTROL_PERIOD {
            return Err(PoseMotionError::Configuration);
        }
        Ok(Self {
            drive,
            options,
            tuning,
            clock: CommandClock::new(CONTROL_PERIOD).ok_or(PoseMotionError::Configuration)?,
            started: Instant::now(),
            budget,
            finished: false,
        })
    }

    async fn tick(&mut self) -> Result<Duration, PoseMotionError<D::Error>> {
        let dt = self.clock.tick().await;
        if competition::mode() != CompetitionMode::Autonomous {
            return Err(PoseMotionError::Cancelled);
        }
        if self.started.elapsed() >= self.budget {
            return Err(PoseMotionError::RouteTimeout);
        }
        if dt.is_zero() || dt > MAX_CONTROL_GAP {
            return Err(PoseMotionError::ControlGap);
        }
        Ok(dt)
    }

    pub(super) async fn run(
        &mut self,
        target: DifferentialTarget,
        timeout: Duration,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let status = match target {
            DifferentialTarget::Turn { .. } | DifferentialTarget::TurnToPoint { .. } => {
                "Autonomous: turn"
            }
            DifferentialTarget::DriveToPoint { .. } => "Autonomous: drive to point",
            DifferentialTarget::DriveDistance { .. } => "Autonomous: drive distance",
            DifferentialTarget::DriveArc { .. } => "Autonomous: drive arc",
        };
        log_row!(
            Debug,
            "MOTION_COMMAND",
            "state=start,command={status},timeout_ms={},route_remaining_ms={}",
            timeout.as_millis(),
            self.budget
                .saturating_sub(self.started.elapsed())
                .as_millis()
        );
        let result = self.run_stage(target, timeout, status).await;
        match &result {
            Ok(()) => log_row!(Debug, "MOTION_COMMAND", "state=complete,command={status}"),
            Err(error) => log_row!(
                Debug,
                "MOTION_COMMAND",
                "state=failed,command={status},reason={}",
                error.reason()
            ),
        }
        result
    }

    async fn run_stage(
        &mut self,
        target: DifferentialTarget,
        timeout: Duration,
        status: &str,
    ) -> Result<(), PoseMotionError<D::Error>> {
        let started = Instant::now();
        let mut controller = DifferentialFeedback::new(self.tuning);
        let mut settled = Duration::ZERO;
        let mut progress = Progress::default();
        loop {
            let dt = self.tick().await?;
            if started.elapsed() >= timeout {
                return Err(PoseMotionError::StageTimeout);
            }
            let Some(feedback) = self.drive.sample().map_err(PoseMotionError::Drive)? else {
                log_row!(
                    Trace,
                    "MOTION_RETRY",
                    "command={status},source=tracking_initializing,elapsed_ms={}",
                    started.elapsed().as_millis()
                );
                self.drive.brake().map_err(PoseMotionError::StopFailed)?;
                controller.pid.reset();
                settled = Duration::ZERO;
                progress = Progress::default();
                continue;
            };
            if !valid_feedback(feedback) {
                return Err(PoseMotionError::InvalidFeedback);
            }
            if matches!(
                target,
                DifferentialTarget::DriveArc { .. }
                    | DifferentialTarget::TurnToPoint { .. }
                    | DifferentialTarget::DriveToPoint { .. }
            ) && !self.drive.pose_trusted()
            {
                return Err(PoseMotionError::UntrustedPose);
            }
            let (wheels, ready) = controller.step(target, feedback, dt, self.options);
            settled = if ready { settled + dt } else { Duration::ZERO };
            let complete = ready && settled >= self.options.settle;
            if complete || wheels == [0.0; 2] {
                self.drive.brake().map_err(PoseMotionError::StopFailed)?;
            } else {
                self.drive
                    .drive_wheels(wheels.map(Velocity::new::<meter_per_second>))
                    .map_err(PoseMotionError::Drive)?;
            }
            log_row!(
                Trace,
                "MOTION_STEP",
                "command={status},elapsed_ms={},dt_us={},x_m={:.5},y_m={:.5},heading_rad={:.5},progress_m=0.00000,left_target_mps={:.5},right_target_mps={:.5},complete={complete},lost=false",
                started.elapsed().as_millis(),
                dt.as_micros(),
                feedback.pose.position.x,
                feedback.pose.position.y,
                feedback.pose.heading.value,
                wheels[0],
                wheels[1]
            );
            self.drive.log_step(self.started.elapsed(), dt);
            if complete {
                return Ok(());
            }
            if progress.stalled(feedback.pose, wheels, dt) {
                return Err(PoseMotionError::NoProgress);
            }
        }
    }

    pub(super) async fn try_distance_reset(
        &mut self,
        timeout: Duration,
    ) -> Result<bool, PoseMotionError<D::Error>> {
        self.drive.brake().map_err(PoseMotionError::StopFailed)?;
        let started = Instant::now();
        loop {
            let dt = self.tick().await?;
            if started.elapsed() >= timeout {
                return Ok(false);
            }
            self.drive.request_distance_reset();
            let feedback = self.drive.sample().map_err(PoseMotionError::Drive)?;
            self.drive.log_step(self.started.elapsed(), dt);
            if let Some(feedback) = feedback {
                if !valid_feedback(feedback) {
                    return Err(PoseMotionError::InvalidFeedback);
                }
                if feedback.reset_accepted {
                    return Ok(true);
                }
            }
        }
    }

    pub(super) fn finish<T>(
        mut self,
        outcome: Result<T, PoseMotionError<D::Error>>,
    ) -> Result<T, PoseMotionError<D::Error>> {
        self.drive.cancel_distance_reset();
        let result = match (outcome, self.drive.brake()) {
            (outcome, Ok(())) => outcome,
            (Ok(_), Err(stop)) => Err(PoseMotionError::StopFailed(stop)),
            (Err(cause), Err(stop)) => Err(PoseMotionError::StopFailedAfter {
                cause: Box::new(cause),
                stop,
            }),
        };
        self.finished = true;
        result
    }
}

impl<D: PoseDrive> Drop for PoseSession<'_, D> {
    fn drop(&mut self) {
        if !self.finished {
            self.drive.cancel_distance_reset();
            let _ = self.drive.brake();
        }
    }
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
    anchor: Option<Pose>,
    elapsed: Duration,
}

impl Progress {
    fn stalled(&mut self, pose: Pose, speeds: [f64; 2], dt: Duration) -> bool {
        if speeds.iter().all(|speed| speed.abs() <= 0.08) {
            *self = Self::default();
            return false;
        }
        let anchor = *self.anchor.get_or_insert(pose);
        if pose.position.distance(anchor.position) > 0.008
            || wrap_heading(pose.heading - anchor.heading).value.abs() > 0.025
        {
            self.anchor = Some(pose);
            self.elapsed = Duration::ZERO;
        } else {
            self.elapsed += dt;
        }
        self.elapsed >= STALL_TIME
    }
}
