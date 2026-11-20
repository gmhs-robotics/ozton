//! Reader for numeric Ozton telemetry. Unknown kinds are ignored by calibration.

use std::{error::Error, io::BufRead};

use ozton_diagnostics::{CSV_HEADER, calibration::CalibrationSample};

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
        let mut fields = line.split(',');
        let parsed = core::array::from_fn::<_, 9, _>(|_| fields.next());
        let row = index + 2;
        let [
            Some(version),
            Some(timestamp),
            Some(kind),
            Some(label),
            Some(a),
            Some(b),
            Some(c),
            Some(_),
            Some(_),
        ] = parsed
        else {
            return Err(format!("malformed telemetry row {row}").into());
        };
        if fields.next().is_some() || version != "1" || timestamp.parse::<u64>().is_err() {
            return Err(format!("malformed telemetry row {row}").into());
        }
        if name.is_some_and(|name| label != name) {
            continue;
        }
        if let Some(sample) =
            CalibrationSample::parse(kind, [a, b, c].map(|field| field.parse().ok()))
                .map_err(|_| format!("missing or invalid calibration sample on row {row}"))?
        {
            data.push_sample(sample);
        }
    }
    Ok(data)
}
