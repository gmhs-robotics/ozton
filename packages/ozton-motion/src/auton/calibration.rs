//! Supervised calibration trials that share the Brain route selector.

use std::{f64::consts::PI, fs::File, time::Duration};

use ozton_diagnostics::{Journal, calibration::CalibrationSample};
use ozton_math::uom::si::f64::{Angle, Length};
use ozton_tracking::localizer::Snapshot;
use vexide::controller::ControllerState;

use super::{ConfiguredChassis, setup::ChassisDriverCommand};

const MAX_HOLD: Duration = Duration::from_secs(8);
const MAX_DRIVE_VOLTS: f64 = 3.0;

/// A known physical reference for a supervised trial.
#[derive(Debug, Clone, Copy)]
pub enum CalibrationTrial {
    /// Hold A and drive a measured straight line. Release A at the mark.
    Straight { distance: Length },
    /// Hold A and turn through a measured angle. Release A at the mark.
    Turn { angle: Angle },
    /// Hold A while stationary to sample a range sensor by index.
    Range { sensor: usize, reference: Length },
    /// Hold A and sweep the forward stick. Record both directions in the same file.
    Feedforward,
}

/// Register a calibration entry and its dedicated SD CSV file.
#[derive(Debug, Clone, Copy)]
pub struct Calibration {
    pub name: &'static str,
    pub file: &'static str,
    pub trial: CalibrationTrial,
}

impl Calibration {
    pub const fn new(name: &'static str, file: &'static str, trial: CalibrationTrial) -> Self {
        Self { name, file, trial }
    }
}

#[derive(Clone, Copy)]
enum Trial {
    Straight(f64),
    Turn(f64),
    Range(usize, f64),
    Feedforward,
}

struct PreparedCalibration {
    pub name: &'static str,
    pub file: &'static str,
    trial: Trial,
}

impl PreparedCalibration {
    pub fn new(config: Calibration, drive: &ConfiguredChassis) -> Option<Self> {
        let positive = |value: f64| value.is_finite() && value > 0.0;
        if config.name.trim().is_empty() || config.file.trim().is_empty() {
            return None;
        }
        let trial = match config.trial {
            CalibrationTrial::Straight { distance } if positive(distance.value) => {
                Trial::Straight(distance.value)
            }
            CalibrationTrial::Turn { angle } if positive(angle.value) => Trial::Turn(angle.value),
            CalibrationTrial::Range { sensor, reference }
                if sensor < drive.range_count() && positive(reference.value) =>
            {
                Trial::Range(sensor, reference.value)
            }
            CalibrationTrial::Feedforward => Trial::Feedforward,
            _ => return None,
        };
        Some(Self {
            name: config.name,
            file: config.file,
            trial,
        })
    }
}

/// A supervised trial for a driver-only choice in the programmed selector.
pub struct CalibrationSession {
    case: PreparedCalibration,
    run: CalibrationRun,
}

impl CalibrationSession {
    pub fn new(config: Calibration, drive: &ConfiguredChassis) -> Option<Self> {
        Some(Self {
            case: PreparedCalibration::new(config, drive)?,
            run: CalibrationRun::new(),
        })
    }

    /// Evaluate one A-held calibration sample and return the drive command to apply.
    pub fn driver(
        &mut self,
        drive: &mut ConfiguredChassis,
        input: &ControllerState,
        dt: Duration,
    ) -> (Option<ChassisDriverCommand>, &'static str) {
        self.run.tick(&self.case, drive, input, dt)
    }

    /// Call on disable or controller loss. A new trial requires A to be released.
    pub fn cancel(&mut self) {
        self.run.cancel();
    }

    /// Save collected samples on disable. Failed saves retain the samples for retry.
    pub fn save(&mut self) -> Option<&'static str> {
        self.run.save()
    }
}

struct CalibrationRun {
    journal: Journal,
    file: Option<&'static str>,
    start: Option<Snapshot>,
    last_range: Option<Duration>,
    require_release: bool,
    samples_this_hold: usize,
}

impl CalibrationRun {
    pub fn new() -> Self {
        Self {
            journal: Journal::new(2048),
            file: None,
            start: None,
            last_range: None,
            require_release: false,
            samples_this_hold: 0,
        }
    }

    pub fn cancel(&mut self) {
        self.start = None;
        self.require_release = true;
    }

    pub fn save(&mut self) -> Option<&'static str> {
        if self.journal.is_empty() {
            self.file = None;
            return None;
        }
        let file = self.file?;
        if File::create(file)
            .and_then(|file| self.journal.write_csv(file))
            .is_err()
        {
            return Some("Calibration save failed");
        }
        self.journal.clear();
        self.file = None;
        self.last_range = None;
        Some("Calibration CSV saved")
    }

    pub fn tick(
        &mut self,
        case: &PreparedCalibration,
        drive: &mut ConfiguredChassis,
        state: &ControllerState,
        dt: Duration,
    ) -> (Option<ChassisDriverCommand>, &'static str) {
        if self.file.is_some_and(|file| file != case.file) {
            return (None, "Save previous calibration first");
        }
        let pressed = state.button_a.is_pressed();
        if !pressed && self.start.is_none() {
            self.require_release = false;
            return (None, "Hold A to sample");
        }
        if self.require_release || dt.is_zero() || dt > drive.settings.max_control_gap {
            return self.abort("Release A to restart");
        }
        let result = drive.chassis.sample(0);
        let Some(sample) = drive.chassis.last_snapshot.filter(|_| result.is_ok()) else {
            return self.abort("Calibration feedback lost");
        };
        if !pressed {
            let Some(start) = self.start.take() else {
                return (None, "Hold A to sample");
            };
            let travel = self.record_travel(
                case.trial,
                start,
                sample,
                drive.settings.wheel_diameter.value,
            );
            return (
                None,
                if travel || self.samples_this_hold > 0 {
                    "Sample saved in memory"
                } else {
                    "No valid sample"
                },
            );
        }
        if self.start.is_none() {
            self.start = Some(sample);
            self.file = Some(case.file);
            self.samples_this_hold = 0;
            drive.chassis.reset_calibration_velocity();
        }
        if self
            .start
            .is_some_and(|start| sample.time.saturating_sub(start.time) >= MAX_HOLD)
        {
            return self.abort("Trial expired. Release A.");
        }
        let [left, right] = match case.trial {
            Trial::Range(index, reference) => {
                if let Some(raw) = drive.chassis.drivetrain.tracking.distances.latest(index)
                    && self.last_range.is_none_or(|last| raw.timestamp > last)
                    && drive
                        .chassis
                        .drivetrain
                        .tracking
                        .localizer
                        .range_config(index)
                        .is_some_and(|config| config.relative_distance(raw, sample.time).is_ok())
                {
                    self.last_range = Some(raw.timestamp);
                    self.samples_this_hold += 1;
                    self.journal.push(
                        CalibrationSample::Range {
                            reference,
                            measured: raw.distance.value,
                        }
                        .event(sample.time, case.name),
                    );
                }
                [0.0; 2]
            }
            Trial::Turn(_) => {
                let steer = state.right_stick.x();
                [steer, -steer]
            }
            Trial::Straight(_) | Trial::Feedforward => [state.left_stick.y(); 2],
        };
        if matches!(case.trial, Trial::Feedforward) {
            let names = ["drive_left", "drive_right"];
            for (name, measurement) in names.into_iter().zip(drive.chassis.feedforward_samples()) {
                if let Some(measurement) = measurement {
                    self.samples_this_hold += 1;
                    self.journal.push(measurement.event(sample.time, name));
                }
            }
        }
        let command = (left.abs().max(right.abs()) > 0.04).then(|| {
            ChassisDriverCommand::Calibration(CalibrationOutput(
                [left, right].map(|input| input * MAX_DRIVE_VOLTS),
            ))
        });
        (command, "Calibration sampling")
    }

    fn abort(&mut self, status: &'static str) -> (Option<ChassisDriverCommand>, &'static str) {
        self.cancel();
        (None, status)
    }

    fn record_travel(
        &mut self,
        trial: Trial,
        start: Snapshot,
        end: Snapshot,
        diameter: f64,
    ) -> bool {
        let ([Some(left_start), Some(right_start)], [Some(left_end), Some(right_end)]) =
            (start.drive, end.drive)
        else {
            return false;
        };
        let [left, right] = [
            left_end.value - left_start.value,
            right_end.value - right_start.value,
        ];
        if !left.is_finite() || !right.is_finite() {
            return false;
        }
        let mut push = |sample| {
            self.journal
                .push(CalibrationSample::event(sample, end.time, "drive"))
        };
        match trial {
            Trial::Straight(meters) if (left + right).abs() > 0.01 => {
                push(CalibrationSample::Straight {
                    meters: meters.copysign(left + right),
                    wheel_turns: [left / (PI * diameter), right / (PI * diameter)],
                });
            }
            Trial::Turn(radians) if (right - left).abs() > 0.01 => {
                let radians = radians.copysign(right - left);
                push(CalibrationSample::Turn {
                    radians,
                    wheel_travel: [left, right],
                });
                if let Some((from, to)) = start.forward.zip(end.forward) {
                    push(CalibrationSample::ForwardOffset {
                        radians,
                        pod_travel: to.value - from.value,
                    });
                }
                if let Some((from, to)) = start.lateral.zip(end.lateral) {
                    push(CalibrationSample::LateralOffset {
                        radians,
                        pod_travel: to.value - from.value,
                    });
                }
            }
            _ => return false,
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use ozton_math::uom::si::length::meter;

    use super::*;

    #[test]
    fn travel_trials_use_signed_references_and_write_pod_offsets() {
        let start = Snapshot {
            drive: [Some(Length::new::<meter>(0.0)); 2],
            forward: Some(Length::new::<meter>(0.0)),
            lateral: Some(Length::new::<meter>(0.0)),
            ..Snapshot::default()
        };
        let end = Snapshot {
            time: Duration::from_millis(100),
            drive: [
                Some(Length::new::<meter>(0.5)),
                Some(Length::new::<meter>(-0.5)),
            ],
            forward: Some(Length::new::<meter>(0.04)),
            lateral: Some(Length::new::<meter>(-0.03)),
            ..start
        };
        let mut run = CalibrationRun::new();
        assert!(run.record_travel(Trial::Turn(2.0), start, end, 0.06));
        let mut csv = Vec::new();
        run.journal.write_csv(&mut csv).unwrap();
        let csv = String::from_utf8(csv).unwrap();
        assert!(csv.contains("turn,drive,-2,0.5,-0.5"));
        assert!(csv.contains("forward-offset,drive,-2,0.04"));
        assert!(csv.contains("lateral-offset,drive,-2,-0.03"));
        assert!(!run.record_travel(Trial::Straight(1.0), start, start, 0.06));
    }
}

/// Bounded motor voltage from a supervised calibration sample.
pub struct CalibrationOutput([f64; 2]);

impl CalibrationOutput {
    pub(super) fn volts(&self) -> [f64; 2] {
        self.0
    }
}
