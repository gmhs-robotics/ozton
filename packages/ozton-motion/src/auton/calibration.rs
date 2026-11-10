//! Supervised calibration trials that share the Brain route selector.

use std::{f64::consts::PI, fs::File, time::Duration};

use ozton_diagnostics::{Journal, calibration::CalibrationSample};
use ozton_tracking::localizer::Snapshot;
use vexide::controller::ControllerState;

use super::{ConfiguredChassis, RangeId};

const MAX_HOLD: Duration = Duration::from_secs(8);
const MAX_DRIVE_VOLTS: f64 = 3.0;

/// A known physical reference for a supervised trial.
#[derive(Debug, Clone, Copy)]
pub enum CalibrationTrial {
    /// Hold A and drive a measured straight line. Release A at the mark.
    Straight { meters: f64 },
    /// Hold A and turn through a measured angle. Release A at the mark.
    Turn { radians: f64 },
    /// Hold A while stationary to sample the named range sensor.
    Range {
        sensor: &'static str,
        reference: f64,
    },
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
    Range(RangeId, f64, &'static str),
    Feedforward,
}

pub(super) struct PreparedCalibration {
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
            CalibrationTrial::Straight { meters } if positive(meters) => Trial::Straight(meters),
            CalibrationTrial::Turn { radians } if positive(radians) => Trial::Turn(radians),
            CalibrationTrial::Range { sensor, reference } if positive(reference) => {
                Trial::Range(drive.range(sensor)?, reference, sensor)
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

pub(super) struct CalibrationRun {
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
    ) -> Option<&'static str> {
        if self.file.is_some_and(|file| file != case.file) {
            let _ = drive.chassis.stop();
            return Some("Save previous calibration first");
        }
        let pressed = state.button_a.is_pressed();
        if !pressed && self.start.is_none() {
            if self.require_release && drive.chassis.stop().is_err() {
                return Some("Calibration output fault");
            }
            self.require_release = false;
            return Some("Hold A to sample");
        }
        if self.require_release || dt.is_zero() || dt > drive.settings.max_control_gap {
            return self.abort(drive, "Release A to restart");
        }
        if !pressed && drive.chassis.stop().is_err() {
            return self.abort(drive, "Calibration output fault");
        }
        let result = drive.chassis.sample(0);
        let Some(sample) = drive.chassis.last_snapshot.filter(|_| result.is_ok()) else {
            return self.abort(drive, "Calibration feedback lost");
        };
        if !pressed {
            let start = self.start.take()?;
            let travel =
                self.record_travel(case.trial, start, sample, drive.settings.wheel_diameter);
            return Some(if travel || self.samples_this_hold > 0 {
                "Sample saved in memory"
            } else {
                "No valid sample"
            });
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
            return self.abort(drive, "Trial expired. Release A.");
        }
        let [left, right] = match case.trial {
            Trial::Range(id, reference, name) => {
                if let Some(raw) = drive.chassis.raw_ranges[id.0]
                    && self.last_range.is_none_or(|last| raw.timestamp > last)
                    && drive
                        .chassis
                        .drivetrain
                        .tracking
                        .range_config(id.0)
                        .is_some_and(|config| config.relative_distance(raw, sample.time).is_ok())
                {
                    self.last_range = Some(raw.timestamp);
                    self.samples_this_hold += 1;
                    self.journal.push(
                        CalibrationSample::Range {
                            reference,
                            measured: raw.distance,
                        }
                        .event(sample.time, name),
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
        let result = if left.abs().max(right.abs()) <= 0.04 {
            drive.chassis.stop().map_err(|_| ())
        } else {
            drive
                .chassis
                .drivetrain
                .model
                .drive_volts(left * MAX_DRIVE_VOLTS, right * MAX_DRIVE_VOLTS)
                .map_err(|_| ())
        };
        if result.is_err() {
            return self.abort(drive, "Calibration output fault");
        }
        Some("Calibration sampling")
    }

    fn abort(
        &mut self,
        drive: &mut ConfiguredChassis,
        status: &'static str,
    ) -> Option<&'static str> {
        let _ = drive.chassis.stop();
        self.cancel();
        Some(status)
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
        let [left, right] = [left_end - left_start, right_end - right_start];
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
                        pod_travel: to - from,
                    });
                }
                if let Some((from, to)) = start.lateral.zip(end.lateral) {
                    push(CalibrationSample::LateralOffset {
                        radians,
                        pod_travel: to - from,
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
    use super::*;

    #[test]
    fn travel_trials_use_signed_references_and_write_pod_offsets() {
        let start = Snapshot {
            drive: [Some(0.0); 2],
            forward: Some(0.0),
            lateral: Some(0.0),
            ..Snapshot::default()
        };
        let end = Snapshot {
            time: Duration::from_millis(100),
            drive: [Some(0.5), Some(-0.5)],
            forward: Some(0.04),
            lateral: Some(-0.03),
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
