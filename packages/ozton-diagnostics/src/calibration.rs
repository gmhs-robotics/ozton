//! Input rows for the host calibration tool.

use std::time::Duration;

use crate::Event;

/// Reference measurements come from a tape measure, fixture, or calibrated sensor.
/// Record each experiment under a separate name when fitting different hardware.
pub enum CalibrationSample {
    Straight {
        meters: f64,
        wheel_turns: [f64; 2],
    },
    Turn {
        radians: f64,
        wheel_travel: [f64; 2],
    },
    ForwardOffset {
        radians: f64,
        pod_travel: f64,
    },
    LateralOffset {
        radians: f64,
        pod_travel: f64,
    },
    Range {
        reference: f64,
        measured: f64,
    },
    Voltage {
        speed: f64,
        acceleration: f64,
        volts: f64,
    },
}

impl CalibrationSample {
    pub fn event(self, time: Duration, name: &'static str) -> Event {
        let missing = f64::NAN;
        let (kind, values) = match self {
            Self::Straight {
                meters,
                wheel_turns: [left, right],
            } => ("straight", [meters, left, right, missing, missing]),
            Self::Turn {
                radians,
                wheel_travel: [left, right],
            } => ("turn", [radians, left, right, missing, missing]),
            Self::ForwardOffset {
                radians,
                pod_travel,
            } => (
                "forward-offset",
                [radians, pod_travel, missing, missing, missing],
            ),
            Self::LateralOffset {
                radians,
                pod_travel,
            } => (
                "lateral-offset",
                [radians, pod_travel, missing, missing, missing],
            ),
            Self::Range {
                reference,
                measured,
            } => (
                "range-calibration",
                [reference, measured, missing, missing, missing],
            ),
            Self::Voltage {
                speed,
                acceleration,
                volts,
            } => ("voltage", [speed, acceleration, volts, missing, missing]),
        };
        Event::new(time, kind, name, values)
    }
}
