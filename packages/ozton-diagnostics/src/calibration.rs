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
    pub fn parse(kind: &str, values: [Option<f64>; 3]) -> Result<Option<Self>, InvalidSample> {
        let [a, b, c] = values;
        let required =
            |value: Option<f64>| value.filter(|value| value.is_finite()).ok_or(InvalidSample);
        let sample = match kind {
            "straight" => Self::Straight {
                meters: required(a)?,
                wheel_turns: [required(b)?, required(c)?],
            },
            "turn" => Self::Turn {
                radians: required(a)?,
                wheel_travel: [required(b)?, required(c)?],
            },
            "forward-offset" => Self::ForwardOffset {
                radians: required(a)?,
                pod_travel: required(b)?,
            },
            "lateral-offset" => Self::LateralOffset {
                radians: required(a)?,
                pod_travel: required(b)?,
            },
            "range-calibration" => Self::Range {
                reference: required(a)?,
                measured: required(b)?,
            },
            "voltage" => Self::Voltage {
                speed: required(a)?,
                acceleration: required(b)?,
                volts: required(c)?,
            },
            _ => return Ok(None),
        };
        Ok(Some(sample))
    }

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InvalidSample;
