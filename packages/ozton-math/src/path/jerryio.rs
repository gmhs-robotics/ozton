//! PATH.JERRYIO's single-path LemLib v0.5 text export.

use alloc::vec::Vec;

use super::{MAX_POINTS, Waypoint};
use crate::geometry::INCH;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImportError {
    MissingEndData,
    InvalidRow(usize),
    TooFewPoints,
    TooManyPoints,
}

/// Reads only robot data before the exact delimiter. Drops the exporter’s
/// 20-inch ghost target, preserving the real terminal position.
pub fn parse(bytes: &[u8]) -> Result<Vec<Waypoint>, ImportError> {
    let mut points = Vec::new();
    let mut terminated = false;
    for (index, bytes) in bytes.split(|byte| *byte == b'\n').enumerate() {
        let line_number = index + 1;
        let line = core::str::from_utf8(bytes)
            .map_err(|_| ImportError::InvalidRow(line_number))?
            .trim();
        if line == "endData" {
            terminated = true;
            break;
        }
        if line.is_empty() {
            continue;
        }
        if points.len() >= MAX_POINTS + 2 {
            return Err(ImportError::TooManyPoints);
        }
        let mut fields = line.split(',');
        let mut number = || -> Result<f64, ImportError> {
            fields
                .next()
                .and_then(|field| field.trim().parse::<f64>().ok())
                .filter(|value| value.is_finite())
                .ok_or(ImportError::InvalidRow(line_number))
        };
        let (x, y, speed) = (number()?, number()?, number()?);
        if fields.next().is_some() || !(0.0..=127.0).contains(&speed) {
            return Err(ImportError::InvalidRow(line_number));
        }
        points.push(Waypoint::new(x * INCH, y * INCH, speed / 127.0));
    }
    if !terminated {
        return Err(ImportError::MissingEndData);
    }
    if let [.., previous, end, ghost] = points.as_slice() {
        let ghost_distance = ghost.position.distance(end.position);
        if ghost.speed_fraction == 0.0
            && (ghost_distance - 20.0 * INCH).abs() <= 0.002 * INCH
            && (previous.position == end.position
                || (end.speed_fraction == 0.0
                    && (end.position - previous.position)
                        .normalize_or_zero()
                        .dot((ghost.position - end.position).normalize_or_zero())
                        > 0.999))
        {
            points.pop();
        }
    }
    points.dedup_by(|next, previous| {
        if next.position == previous.position {
            previous.speed_fraction = previous.speed_fraction.min(next.speed_fraction);
            true
        } else {
            false
        }
    });
    if points.len() < 2 {
        return Err(ImportError::TooFewPoints);
    }
    if points.len() > MAX_POINTS {
        return Err(ImportError::TooManyPoints);
    }
    Ok(points)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trims_ghost_and_ignores_editor_bytes() {
        let points = parse(
            b"0, 0, 127\r\n0, 6, 100\r\n0, 12, 0\r\n0, 12, 127\r\n0, 32, 0\r\nendData\r\n\xff",
        )
        .unwrap();
        assert_eq!(points.len(), 3);
        assert_eq!(points[0].speed_fraction, 1.0);
        assert_eq!(points[2], Waypoint::new(0.0, 12.0 * INCH, 0.0));
        assert_eq!(parse(b"0,0,NaN\nendData"), Err(ImportError::InvalidRow(1)));
    }
}

/// A chain keeps the authored junction and profiles through it in one motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChainError {
    Empty,
    Import { segment: usize, error: ImportError },
    Junction(usize),
    TooManyPoints,
}

pub fn parse_chain(segments: &[&[u8]]) -> Result<Vec<Waypoint>, ChainError> {
    let mut chain: Vec<Waypoint> = Vec::new();
    for (index, bytes) in segments.iter().enumerate() {
        let points = parse(bytes).map_err(|error| ChainError::Import {
            segment: index,
            error,
        })?;
        if chain.is_empty() {
            chain = points;
            continue;
        }
        let junction = chain.len() - 1;
        let previous = chain[junction - 1];
        let end = chain[junction].position;
        let outgoing = points[1].position - points[0].position;
        let incoming = end - previous.position;
        if end.distance(points[0].position) > 0.002
            || incoming
                .normalize_or_zero()
                .dot(outgoing.normalize_or_zero())
                < 0.95
        {
            return Err(ChainError::Junction(index));
        }
        if chain.len() + points.len() - 1 > MAX_POINTS {
            return Err(ChainError::TooManyPoints);
        }
        chain[junction].speed_fraction = previous
            .speed_fraction
            .min(points[0].speed_fraction.max(points[1].speed_fraction));
        if chain[junction].speed_fraction <= 0.0 {
            return Err(ChainError::Junction(index));
        }
        chain.extend_from_slice(&points[1..]);
    }
    if chain.is_empty() {
        return Err(ChainError::Empty);
    }
    Ok(chain)
}
