//! Vexide differential chassis for the programmed autonomous runner.

use std::time::Duration;

use ozton_control::axis::{AxisController, AxisSample, Target};
use ozton_diagnostics::{Event, Journal, calibration::CalibrationSample};
use ozton_drivetrain::{
    Drivetrain,
    model::{Differential, Tank, WheelFeedback},
};
use ozton_math::geometry::Pose;
use ozton_tracking::{
    TracksForwardTravel,
    localizer::{
        Correction, Fault, Localizer, MAX_RANGES, RangeObservation, Snapshot, Update, vex::Sensors,
    },
};
use vexide::smart::PortError;

use super::{Feedback, RangeFeedback};

#[derive(Debug)]
pub enum ChassisError {
    Tracking(Fault),
    Output(PortError),
    WheelFeedback,
    InvalidCommand,
}

/// Native wheel velocity control, or shared PID/feedforward controllers in SI units.
#[derive(Clone, Copy)]
pub enum WheelControl {
    Native,
    Voltage([AxisController; 2]),
}

pub struct Chassis<'a> {
    pub drivetrain: Drivetrain<Differential, Localizer<'a>>,
    pub sensors: Sensors,
    pub journal: Journal,
    control: WheelControl,
    wheels: [Option<WheelFeedback>; 2],
    target: [f64; 2],
    range: [Option<RangeFeedback>; MAX_RANGES],
    time: Duration,
    tracking_status: &'static str,
    calibration_velocity: [Option<(f64, Duration)>; 2],
    pub(super) last_snapshot: Option<Snapshot>,
    pub(super) raw_ranges: [Option<RangeObservation>; MAX_RANGES],
}

impl<'a> Chassis<'a> {
    pub fn new(
        drivetrain: Drivetrain<Differential, Localizer<'a>>,
        sensors: Sensors,
        control: WheelControl,
        journal_capacity: usize,
    ) -> Option<Self> {
        if matches!(control, WheelControl::Native) && drivetrain.model.wheel_speed_limit().is_none()
        {
            return None;
        }
        Some(Self {
            drivetrain,
            sensors,
            journal: Journal::new(journal_capacity),
            control,
            wheels: [None; 2],
            target: [0.0; 2],
            range: [None; MAX_RANGES],
            time: Duration::ZERO,
            tracking_status: "Tracking initializing",
            calibration_velocity: [None; 2],
            last_snapshot: None,
            raw_ranges: [None; MAX_RANGES],
        })
    }

    /// Sample once per tick. Range bits mark mechanism-dependent occlusions.
    pub fn sample(&mut self, occluded: u8) -> Result<Option<Feedback>, ChassisError> {
        self.wheels = self.drivetrain.model.feedback();
        let (snapshot, ranges) = self.sensors.capture(
            self.wheels.map(|wheel| wheel.map(|wheel| wheel.position)),
            occluded,
        );
        self.time = snapshot.time;
        self.last_snapshot = Some(snapshot);
        self.raw_ranges = ranges;
        let update = match self.drivetrain.tracking.update(snapshot, &ranges) {
            Ok(update) => update,
            Err(Fault::Initializing) => return Ok(None),
            Err(fault) => {
                self.tracking_status = "Tracking unavailable";
                self.range.fill(None);
                return Err(ChassisError::Tracking(fault));
            }
        };
        self.tracking_status = if update.distance_reset {
            "Distance reset accepted"
        } else {
            self.drivetrain.tracking.estimator_name()
        };
        for (index, latest) in self.range.iter_mut().enumerate() {
            let candidate = ranges[index].and_then(|reading| {
                let config = self.drivetrain.tracking.range_config(index)?;
                let distance = config.relative_distance(reading, self.time).ok()?;
                Some(RangeFeedback {
                    distance,
                    timestamp: reading.timestamp,
                })
            });
            *latest = match (candidate, *latest) {
                (Some(new), Some(previous)) if new.timestamp < previous.timestamp => None,
                (candidate, _) => candidate,
            };
        }
        self.log_tracking(update);
        Ok(Some(Feedback {
            pose: update.pose,
            forward_velocity: update.forward_velocity,
            forward_travel: self.drivetrain.tracking.forward_travel(),
            angular_velocity: update.angular_velocity,
            ranges: self.range,
            reset_accepted: update.distance_reset,
        }))
    }

    pub fn drive(&mut self, target: [f64; 2], dt: Duration) -> Result<(), ChassisError> {
        if dt.is_zero() || target.iter().any(|speed| !speed.is_finite()) {
            let _ = self.stop();
            return Err(ChassisError::InvalidCommand);
        }
        let previous = self.target;
        self.target = target;
        let result = match &mut self.control {
            WheelControl::Native => {
                let limit = self
                    .drivetrain
                    .model
                    .wheel_speed_limit()
                    .expect("validated at construction");
                let power = target.map(|speed| speed / limit);
                self.drivetrain
                    .model
                    .drive_tank(power[0], power[1])
                    .map_err(ChassisError::Output)
            }
            WheelControl::Voltage(controllers) => {
                let voltage = (|| {
                    let mut voltage = [0.0; 2];
                    for (index, controller) in controllers.iter_mut().enumerate() {
                        let feedback = self.wheels[index].ok_or(ChassisError::WheelFeedback)?;
                        let sample = AxisSample {
                            position: feedback.position,
                            velocity: feedback.velocity,
                        };
                        let acceleration = (target[index] - previous[index]) / dt.as_secs_f64();
                        voltage[index] = controller
                            .update(
                                sample,
                                Target::Velocity {
                                    velocity: target[index],
                                    acceleration,
                                },
                                dt,
                            )
                            .ok_or(ChassisError::WheelFeedback)?;
                    }
                    Ok(voltage)
                })();
                voltage.and_then(|voltage| {
                    self.journal.push(Event::new(
                        self.time,
                        "drive",
                        "requested_volts",
                        [voltage[0], voltage[1], f64::NAN, f64::NAN, f64::NAN],
                    ));
                    self.drivetrain
                        .model
                        .drive_volts(voltage[0], voltage[1])
                        .map_err(ChassisError::Output)
                })
            }
        };
        if result.is_err() {
            let _ = self.stop();
        }
        result
    }

    pub fn stop(&mut self) -> Result<(), ChassisError> {
        self.target = [0.0; 2];
        if let WheelControl::Voltage(controllers) = &mut self.control {
            controllers.iter_mut().for_each(AxisController::reset);
        }
        self.drivetrain
            .model
            .stop_now()
            .map_err(ChassisError::Output)
    }

    pub fn set_pose(&mut self, pose: Pose) -> Result<(), ChassisError> {
        self.range.fill(None);
        self.drivetrain
            .tracking
            .set_pose(pose)
            .then_some(())
            .ok_or(ChassisError::InvalidCommand)
    }

    pub fn tracking_status(&self) -> &'static str {
        self.tracking_status
    }

    /// Sample applied voltage during a separately supervised characterization run.
    /// Call after sample() on each tick of the test. This method never moves a motor.
    pub fn log_feedforward_calibration(&mut self, names: [&'static str; 2]) {
        if !self.journal.enabled() {
            return;
        }
        let samples = self.feedforward_samples();
        for (name, sample) in names.into_iter().zip(samples) {
            if let Some(sample) = sample {
                self.journal.push(sample.event(self.time, name));
            }
        }
    }

    pub(super) fn reset_calibration_velocity(&mut self) {
        self.calibration_velocity = [None; 2];
    }

    pub(super) fn feedforward_samples(&mut self) -> [Option<CalibrationSample>; 2] {
        let voltage = self.drivetrain.model.measured_voltage();
        [0, 1].map(|index| {
            let speed = self.wheels[index].map(|wheel| wheel.velocity);
            let previous = core::mem::replace(
                &mut self.calibration_velocity[index],
                speed.map(|value| (value, self.time)),
            );
            if let (Some(speed), Some((previous_speed, previous_time)), Some(volts)) =
                (speed, previous, voltage[index])
                && let Some(dt) = self.time.checked_sub(previous_time)
                && !dt.is_zero()
                && dt <= Duration::from_millis(100)
            {
                let acceleration = (speed - previous_speed) / dt.as_secs_f64();
                Some(CalibrationSample::Voltage {
                    speed,
                    acceleration,
                    volts,
                })
            } else {
                None
            }
        })
    }

    pub fn request_distance_reset(&mut self) {
        self.drivetrain.tracking.request_distance_reset();
    }

    pub fn log_step(&mut self, elapsed: Duration, dt: Duration, progress: f64, wheels: [f64; 2]) {
        self.journal.push(Event::new(
            self.time,
            "auton",
            "step",
            [
                elapsed.as_secs_f64(),
                dt.as_secs_f64(),
                progress,
                wheels[0],
                wheels[1],
            ],
        ));
    }

    fn log_tracking(&mut self, update: Update) {
        if !self.journal.enabled() {
            return;
        }
        let missing = f64::NAN;
        let raw = update.raw;
        self.journal.push(Event::new(
            self.time,
            "tracking",
            "sensors",
            [
                raw.drive[0].unwrap_or(missing),
                raw.drive[1].unwrap_or(missing),
                raw.forward.unwrap_or(missing),
                raw.lateral.unwrap_or(missing),
                raw.imu.unwrap_or(missing),
            ],
        ));
        self.journal.push(Event::new(
            self.time,
            "drive",
            "velocity",
            [
                self.wheels[0].map_or(missing, |wheel| wheel.velocity),
                self.wheels[1].map_or(missing, |wheel| wheel.velocity),
                self.target[0],
                self.target[1],
                missing,
            ],
        ));
        for (label, pose) in [
            ("odometry", Some(update.odometry)),
            ("prediction", Some(update.predicted)),
            ("selected", Some(update.pose)),
            ("candidate", update.candidate),
        ] {
            if let Some(pose) = pose {
                self.journal.push(Event::new(
                    self.time,
                    "tracking",
                    label,
                    [
                        pose.position.x,
                        pose.position.y,
                        pose.heading,
                        update.forward_velocity,
                        update.angular_velocity,
                    ],
                ));
            }
        }
        self.journal.push(Event::new(
            self.time,
            "tracking",
            match update.correction {
                Correction::None => "no_correction",
                Correction::Accepted => "accepted",
                Correction::Rejected => "rejected",
                Correction::Shadow => "shadow",
            },
            [missing; 5],
        ));
        for (index, report) in update.ranges.iter().enumerate() {
            let Some(report) = report else {
                continue;
            };
            use ozton_math::localization::range::Rejection;
            let reason = match report.constraint {
                Ok(_) => "accepted",
                Err(Rejection::Duplicate) => continue,
                Err(Rejection::Missing) => "missing",
                Err(Rejection::Occluded) => "occluded",
                Err(Rejection::OutOfRange) => "out_of_range",
                Err(Rejection::LowConfidence) => "low_confidence",
                Err(Rejection::Stale) => "stale",
                Err(Rejection::NoSurface) => "no_surface",
                Err(Rejection::WrongSurface) => "wrong_surface",
                Err(Rejection::NearEdge) => "near_edge",
                Err(Rejection::Grazing) => "grazing",
                Err(Rejection::Residual) => "residual",
                Err(Rejection::Uncertain) => "uncertain",
            };
            self.journal.push(Event::new(
                self.time,
                "range",
                reason,
                [
                    index as f64,
                    report
                        .observation
                        .map_or(missing, |reading| reading.distance),
                    report
                        .observation
                        .map_or(missing, |reading| reading.confidence),
                    report
                        .constraint
                        .map_or(missing, |constraint| constraint.hit.distance),
                    report
                        .constraint
                        .map_or(missing, |constraint| constraint.residual),
                ],
            ));
        }
    }
}
