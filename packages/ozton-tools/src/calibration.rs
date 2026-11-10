//! Least-squares calibration and reproducible fit diagnostics.

use std::{error::Error, f64::consts::PI, io::Write};

pub const COLUMNS: &str = "kind               a                       b                       c
straight           reference distance (m)  left wheel turns        right wheel turns
turn               reference heading (rad) left wheel travel (m)   right wheel travel (m)
lateral-offset     reference heading (rad) lateral wheel travel (m)
forward-offset     reference heading (rad) forward wheel travel (m)
range-calibration  reference range (m)     measured range (m)
voltage            measured speed (m/s)    acceleration (m/s²)     applied voltage (V)
Use the name column to isolate a sensor, motor, or trial.";

#[derive(Default)]
pub struct Dataset {
    straight: Vec<[f64; 3]>,
    turn: Vec<[f64; 3]>,
    lateral: Vec<[f64; 3]>,
    forward: Vec<[f64; 3]>,
    range: Vec<[f64; 3]>,
    voltage: Vec<[f64; 3]>,
}

impl Dataset {
    pub fn push(&mut self, kind: &str, values: [f64; 3]) {
        match kind {
            "straight" => &mut self.straight,
            "turn" => &mut self.turn,
            "lateral-offset" => &mut self.lateral,
            "forward-offset" => &mut self.forward,
            "range-calibration" => &mut self.range,
            "voltage" => &mut self.voltage,
            _ => return,
        }
        .push(values);
    }
}

struct Estimate {
    name: &'static str,
    value: f64,
    samples: usize,
    rmse: f64,
}

pub fn report(data: Dataset, mut writer: impl Write) -> Result<(), Box<dyn Error>> {
    let mut estimates = Vec::new();
    for (rows, name, sample, positive) in [
        (
            &data.straight,
            "LEFT_WHEEL_DIAMETER_M",
            (|[distance, turns, _]| (turns * PI, distance)) as fn([f64; 3]) -> (f64, f64),
            true,
        ),
        (
            &data.straight,
            "RIGHT_WHEEL_DIAMETER_M",
            |[distance, _, turns]| (turns * PI, distance),
            true,
        ),
        (
            &data.turn,
            "TRACK_WIDTH_M",
            |[angle, left, right]| (angle, right - left),
            true,
        ),
        (
            &data.lateral,
            "LATERAL_WHEEL_OFFSET_M",
            |[angle, travel, _]| (angle, travel),
            false,
        ),
        (
            &data.forward,
            "FORWARD_WHEEL_OFFSET_M",
            |[angle, travel, _]| (-angle, travel),
            false,
        ),
    ] {
        if rows.is_empty() {
            continue;
        }
        let (value, rmse) = slope(rows.iter().copied().map(sample))
            .filter(|(value, _)| !positive || *value > 0.0)
            .ok_or_else(|| format!("insufficient or inconsistent samples for {name}"))?;
        estimates.push(Estimate {
            name,
            value,
            samples: rows.len(),
            rmse,
        });
    }
    if !data.range.is_empty() {
        let count = data.range.len();
        if count < 3 || data.range.iter().any(|row| row[0] <= 0.0 || row[1] <= 0.0) {
            return Err("range calibration requires three positive pairs".into());
        }
        let error = |row: &[f64; 3]| row[1] - row[0];
        let mean = data.range.iter().map(error).sum::<f64>() / count as f64;
        let sum_squared = data
            .range
            .iter()
            .map(|row| (error(row) - mean).powi(2))
            .sum::<f64>();
        let sigma = (sum_squared / (count - 1) as f64).sqrt().max(0.001);
        let rmse = (sum_squared / count as f64).sqrt();
        estimates.push(Estimate {
            name: "RANGE_BIAS_M",
            value: mean,
            samples: count,
            rmse,
        });
        estimates.push(Estimate {
            name: "RANGE_SIGMA_M",
            value: sigma,
            samples: count,
            rmse,
        });
    }
    if !data.voltage.is_empty() {
        let (coefficients, samples, rmse) = feedforward(&data.voltage).ok_or(
            "feedforward needs independent acceleration and speed samples in both directions",
        )?;
        for (name, value) in ["KS_VOLTS", "KV_VOLTS_PER_MPS", "KA_VOLTS_PER_MPS2"]
            .into_iter()
            .zip(coefficients)
        {
            estimates.push(Estimate {
                name,
                value,
                samples,
                rmse,
            });
        }
    }
    if estimates.is_empty() {
        return Err("no calibration samples found".into());
    }
    if estimates
        .iter()
        .any(|estimate| !estimate.value.is_finite() || !estimate.rmse.is_finite())
    {
        return Err("calibration values overflowed".into());
    }
    for Estimate {
        name,
        value,
        samples,
        rmse,
    } in estimates
    {
        writeln!(
            writer,
            "const {name}: f64 = {value:?}; // n={samples}, RMSE={rmse:.6}"
        )?;
    }
    Ok(())
}

fn slope(samples: impl Iterator<Item = (f64, f64)> + Clone) -> Option<(f64, f64)> {
    let (mut cross, mut norm, mut count) = (0.0, 0.0, 0);
    for (x, y) in samples.clone() {
        cross += x * y;
        norm += x * x;
        count += 1;
    }
    if count < 2 || norm <= 1e-9 || !norm.is_finite() {
        return None;
    }
    let value = cross / norm;
    let rmse = (samples.map(|(x, y)| (y - value * x).powi(2)).sum::<f64>() / count as f64).sqrt();
    (value.is_finite() && rmse.is_finite()).then_some((value, rmse))
}

/// Solves the column-normalized least-squares system by modified Gram-Schmidt.
fn feedforward(rows: &[[f64; 3]]) -> Option<([f64; 3], usize, f64)> {
    let mut columns = [Vec::new(), Vec::new(), Vec::new()];
    let mut volts = Vec::new();
    for &[speed, acceleration, voltage] in rows {
        if speed.abs() <= 0.01 {
            continue;
        }
        columns[0].push(speed.signum());
        columns[1].push(speed);
        columns[2].push(acceleration);
        volts.push(voltage);
    }
    let count = volts.len();
    if count < 6 || !columns[0].contains(&-1.0) || !columns[0].contains(&1.0) {
        return None;
    }
    let dot = |left: &[f64], right: &[f64]| left.iter().zip(right).map(|(a, b)| a * b).sum::<f64>();
    let scales = columns.each_ref().map(|column| dot(column, column).sqrt());
    if scales
        .iter()
        .any(|scale| !scale.is_finite() || *scale < 1e-9)
    {
        return None;
    }
    for (column, scale) in columns.iter_mut().zip(scales) {
        column.iter_mut().for_each(|value| *value /= scale);
    }
    let mut r = [[0.0; 3]; 3];
    let mut response = [0.0; 3];
    for i in 0..3 {
        r[i][i] = dot(&columns[i], &columns[i]).sqrt();
        if r[i][i] < 1e-4 {
            return None;
        }
        columns[i].iter_mut().for_each(|value| *value /= r[i][i]);
        response[i] = dot(&columns[i], &volts);
        let (basis, remaining) = columns.split_at_mut(i + 1);
        for (offset, column) in remaining.iter_mut().enumerate() {
            let j = i + 1 + offset;
            r[i][j] = dot(&basis[i], column);
            for (value, component) in column.iter_mut().zip(&basis[i]) {
                *value -= r[i][j] * component;
            }
        }
    }
    let mut coefficients = [0.0; 3];
    for i in (0..3).rev() {
        let projection = (i + 1..3).map(|j| r[i][j] * coefficients[j]).sum::<f64>();
        coefficients[i] = (response[i] - projection) / r[i][i];
    }
    for (coefficient, scale) in coefficients.iter_mut().zip(scales) {
        *coefficient /= scale;
        if !coefficient.is_finite() || *coefficient < -1e-8 {
            return None;
        }
        *coefficient = coefficient.max(0.0);
    }
    let [ks, kv, ka] = coefficients;
    let residuals = rows
        .iter()
        .filter(|row| row[0].abs() > 0.01)
        .map(|[speed, acceleration, voltage]| {
            (voltage - (ks * speed.signum() + kv * speed + ka * acceleration)).powi(2)
        })
        .sum::<f64>();
    Some((coefficients, count, (residuals / count as f64).sqrt()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recovers_feedforward_and_rejects_unidentifiable_input() {
        let rows = [
            (-1.0_f64, -2.0),
            (-1.8, 2.0),
            (-0.3, 0.4),
            (0.5, 1.0),
            (1.5, 0.3),
            (0.6, -1.3),
        ]
        .map(|(v, a)| [v, a, v.signum() * 0.4 + v * 2.0 + a * 0.25]);
        let (coefficients, count, residual) = feedforward(&rows).unwrap();
        assert_eq!(count, 6);
        assert!(residual < 1e-10);
        for (actual, expected) in coefficients.into_iter().zip([0.4, 2.0, 0.25]) {
            assert!((actual - expected).abs() < 1e-10);
        }
        assert!(feedforward(&rows.map(|[v, _, volts]| [v, 2.0 * v, volts])).is_none());
    }
}
