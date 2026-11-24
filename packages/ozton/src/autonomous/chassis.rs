//! Vexide differential chassis for the programmed autonomous runner.
use std::time::Duration;

use ozton_control::axis::{AxisController, AxisSample, Linear, Target};
#[cfg(feature = "logging")]
use ozton_diagnostics::{Event, serial};
use ozton_diagnostics::{Journal, calibration::CalibrationSample};
use ozton_drivetrain::{
    Drivetrain,
    model::{Differential, Drive, WheelFeedback, WheelSpeedError},
};
#[cfg(feature = "logging")]
use ozton_localization::localizer::{Correction, Update};
use ozton_localization::localizer::{
    Fault, Filter, Localizer, MAX_RANGES, PoseEstimator, Snapshot,
};
use ozton_math::{
    geometry::Pose,
    uom::si::{
        acceleration::meter_per_second_squared,
        angular_velocity::radian_per_second,
        electric_potential::volt,
        f64::{Acceleration, AngularVelocity, ElectricPotential, Length, Velocity},
        length::meter,
        velocity::meter_per_second,
    },
};
use ozton_tracking::{TracksForwardTravel, range::Distances, sampling::SharedSensors};
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
#[derive(Clone)]
pub enum WheelControl {
    Native,
    Voltage(Box<[AxisController<Linear>; 2]>),
}

impl WheelControl {
    /// Allocate the two controllers once, during configuration.
    pub fn voltage(controllers: [AxisController<Linear>; 2]) -> Self {
        Self::Voltage(Box::new(controllers))
    }
}

pub(super) struct ChassisSensors {
    pub(super) drive_encoders: ozton_tracking::odometric::DifferentialTracking,
    pub(super) sensors: SharedSensors,
    pub(super) distances: Distances,
}

impl ozton_tracking::TrackingSystem for ChassisSensors {
    fn drive_wheels(&self) -> Option<&ozton_tracking::odometric::DifferentialTracking> {
        Some(&self.drive_encoders)
    }

    fn drive_wheels_mut(&mut self) -> Option<&mut ozton_tracking::odometric::DifferentialTracking> {
        Some(&mut self.drive_encoders)
    }
}

pub struct Chassis<'a, E: PoseEstimator = Filter> {
    pub(super) drivetrain: Drivetrain<Differential, ChassisSensors, Localizer<'a, E>>,
    pub journal: Journal,
    control: WheelControl,
    wheels: [Option<WheelFeedback>; 2],
    target: [f64; 2],
    range: [Option<RangeFeedback>; MAX_RANGES],
    time: Duration,
    tracking_status: &'static str,
    calibration_velocity: [Option<(f64, Duration)>; 2],
    pub(super) last_snapshot: Option<Snapshot>,
}

impl<'a, E: PoseEstimator> Chassis<'a, E> {
    pub(super) fn new(
        drivetrain: Drivetrain<Differential, ChassisSensors, Localizer<'a, E>>,
        control: WheelControl,
        journal_capacity: usize,
    ) -> Option<Self> {
        if drivetrain.tracking.distances.len() != drivetrain.localization.range_count() {
            return None;
        }
        Some(Self {
            drivetrain,
            journal: Journal::new(if cfg!(feature = "logging") {
                journal_capacity
            } else {
                0
            }),
            control,
            wheels: [None; 2],
            target: [0.0; 2],
            range: [None; MAX_RANGES],
            time: Duration::ZERO,
            tracking_status: "Tracking initializing",
            calibration_velocity: [None; 2],
            last_snapshot: None,
        })
    }

    pub fn pose_trusted(&self) -> bool {
        self.drivetrain.localization.pose_trust()
            == ozton_localization::localizer::PoseTrust::Trusted
    }

    pub fn cancel_distance_reset(&mut self) {
        self.drivetrain.localization.cancel_distance_reset();
    }

    /// Sample once per tick. Range bits mark mechanism-dependent occlusions.
    pub fn sample(&mut self, occluded: u8) -> Result<Option<Feedback>, ChassisError> {
        self.wheels = self.drivetrain.sample_feedback();
        self.drivetrain.tracking.sensors.refresh_orientation();
        let (snapshot, ranges) = self.drivetrain.tracking.sensors.capture_with_distances(
            &mut self.drivetrain.tracking.distances,
            self.wheels.map(|wheel| wheel.map(|wheel| wheel.position)),
            occluded,
        );
        self.time = snapshot.time;
        self.last_snapshot = Some(snapshot);
        let update = match self.drivetrain.localization.update(snapshot, &ranges) {
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
            self.drivetrain.localization.estimator_name()
        };
        for (index, latest) in self.range.iter_mut().enumerate() {
            let candidate = ranges[index].and_then(|reading| {
                let config = self.drivetrain.localization.range_config(index)?;
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
        #[cfg(feature = "logging")]
        self.log_tracking(update);
        Ok(Some(Feedback {
            time: snapshot.time,
            pose: update.pose,
            forward_velocity: Velocity::new::<meter_per_second>(update.forward_velocity),
            lateral_velocity: Velocity::new::<meter_per_second>(update.lateral_velocity),
            forward_travel: Length::new::<meter>(self.drivetrain.localization.forward_travel()),
            angular_velocity: AngularVelocity::new::<radian_per_second>(update.angular_velocity),
            ranges: self.range,
            reset_accepted: update.distance_reset,
        }))
    }

    pub fn drive(&mut self, target: [Velocity; 2], dt: Duration) -> Result<(), ChassisError> {
        let target = target.map(|speed| speed.value);
        if dt.is_zero() || target.iter().any(|speed| !speed.is_finite()) {
            let _ = self.stop();
            return Err(ChassisError::InvalidCommand);
        }
        let previous = self.target;
        self.target = target;
        let result = match &mut self.control {
            WheelControl::Native => {
                let [left, right] = target.map(Velocity::new::<meter_per_second>);
                self.drivetrain
                    .drive_wheel_speeds([left, right])
                    .map_err(|error| match error {
                        WheelSpeedError::Port(port) => ChassisError::Output(port),
                        _ => ChassisError::InvalidCommand,
                    })
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
                                    velocity: Velocity::new::<meter_per_second>(target[index]),
                                    acceleration: Acceleration::new::<meter_per_second_squared>(
                                        acceleration,
                                    ),
                                },
                                dt,
                            )
                            .ok_or(ChassisError::WheelFeedback)?
                            .value;
                    }
                    Ok(voltage)
                })();
                voltage.and_then(|voltage| {
                    #[cfg(feature = "logging")]
                    self.record_event(Event::new(
                        self.time,
                        "drive",
                        "requested_volts",
                        [voltage[0], voltage[1], f64::NAN, f64::NAN, f64::NAN],
                    ));
                    self.drivetrain
                        .drive_volts(voltage.map(ElectricPotential::new::<volt>))
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
            .localization
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
        #[cfg(not(feature = "logging"))]
        let _ = names;
        #[cfg(feature = "logging")]
        if self.journal.enabled() || serial::sd_enabled() {
            for (name, sample) in names.into_iter().zip(self.feedforward_samples()) {
                if let Some(sample) = sample {
                    self.record_event(sample.event(self.time, name));
                }
            }
        }
    }

    pub(super) fn reset_calibration_velocity(&mut self) {
        self.calibration_velocity = [None; 2];
    }

    pub(super) fn feedforward_samples(&mut self) -> [Option<CalibrationSample>; 2] {
        let voltage = self.drivetrain.measured_voltage();
        [0, 1].map(|index| {
            let speed = self.wheels[index].map(|wheel| wheel.velocity.value);
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
                    volts: volts.value,
                })
            } else {
                None
            }
        })
    }

    pub fn request_distance_reset(&mut self) {
        self.drivetrain.localization.request_distance_reset();
    }

    pub fn log_step(
        &mut self,
        elapsed: Duration,
        dt: Duration,
        progress: Length,
        wheels: [Velocity; 2],
    ) {
        #[cfg(not(feature = "logging"))]
        let _ = (elapsed, dt, progress, wheels);
        #[cfg(feature = "logging")]
        self.record_event(Event::new(
            self.time,
            "auton",
            "step",
            [
                elapsed.as_secs_f64(),
                dt.as_secs_f64(),
                progress.value,
                wheels[0].value,
                wheels[1].value,
            ],
        ));
    }

    #[cfg(feature = "logging")]
    fn record_event(&mut self, event: Event) {
        if serial::sd_enabled() {
            let [a, b, c, d, e] = event.values.map(|value| value.unwrap_or(f64::NAN));
            serial::row(
                serial::Level::Trace,
                event.time,
                "CHASSIS",
                format_args!(
                    "kind={},name={},a={a},b={b},c={c},d={d},e={e}",
                    serial::field(event.kind),
                    serial::field(event.name)
                ),
            );
        }
        self.journal.push(event);
    }

    #[cfg(feature = "logging")]
    fn log_tracking(&mut self, update: Update) {
        if !self.journal.enabled() && !serial::sd_enabled() {
            return;
        }
        let missing = f64::NAN;
        let raw = update.raw;
        self.record_event(Event::new(
            self.time,
            "tracking",
            "sensors",
            [
                raw.drive[0].map_or(missing, |value| value.value),
                raw.drive[1].map_or(missing, |value| value.value),
                raw.forward.map_or(missing, |value| value.value),
                raw.lateral.map_or(missing, |value| value.value),
                raw.imu.map_or(missing, |value| value.value),
            ],
        ));
        self.record_event(Event::new(
            self.time,
            "drive",
            "velocity",
            [
                self.wheels[0].map_or(missing, |wheel| wheel.velocity.value),
                self.wheels[1].map_or(missing, |wheel| wheel.velocity.value),
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
                self.record_event(Event::new(
                    self.time,
                    "tracking",
                    label,
                    [
                        pose.position.x,
                        pose.position.y,
                        pose.heading.value,
                        update.forward_velocity,
                        update.angular_velocity,
                    ],
                ));
            }
        }
        self.record_event(Event::new(
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
                Err(Rejection::Uncertain) => "uncertain",
            };
            self.record_event(Event::new(
                self.time,
                "range",
                reason,
                [
                    index as f64,
                    report
                        .observation
                        .map_or(missing, |reading| reading.distance.value),
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
