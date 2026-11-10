//! Reader for numeric Ozton telemetry. Unknown kinds are ignored by calibration.

use std::{error::Error, io::BufRead};

use ozton_diagnostics::CSV_HEADER;

use crate::calibration::Dataset;

pub fn read(reader: impl BufRead, name: Option<&str>) -> Result<Dataset, Box<dyn Error>> {
    let mut lines = reader.lines();
    if lines.next().transpose()?.as_deref() != Some(CSV_HEADER) {
        return Err("expected Ozton CSV v1 header".into());
    }
    let mut data = Dataset::default();
    for (index, line) in lines.enumerate() {
        let line = line?;
        if line.is_empty() {
            continue;
        }
        let fields: Vec<_> = line.split(',').collect();
        let row = index + 2;
        if fields.len() != 9 || fields[0] != "1" || fields[1].parse::<u64>().is_err() {
            return Err(format!("malformed telemetry row {row}").into());
        }
        if name.is_some_and(|name| fields[3] != name) {
            continue;
        }
        let count = match fields[2] {
            "straight" | "voltage" => 3,
            "turn" => 3,
            "lateral-offset" | "forward-offset" | "range-calibration" => 2,
            _ => continue,
        };
        let mut values = [0.0; 3];
        for (value, field) in values[..count].iter_mut().zip(&fields[4..4 + count]) {
            *value = field
                .parse::<f64>()
                .ok()
                .filter(|value| value.is_finite())
                .ok_or_else(|| format!("missing or invalid calibration sample on row {row}"))?;
        }
        data.push(fields[2], values);
    }
    Ok(data)
}
