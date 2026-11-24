//! Bounded, clearance-aware differential pose planning in a rectangular field.
use alloc::vec::Vec;
use crate::geometry::{Pose, direction, wrap_heading};
use super::{Limits, Path, Waypoint, clearance::RectangularClearance};

/// A forward or backward trajectory with the lowest predicted time among tested candidates.
pub struct Maneuver {
    pub path: Path,
    pub reverse: bool,
}

/// Validate the oriented footprint over each interpolated segment, including reverse travel.
pub fn clear(path: &Path, reverse: bool, field: RectangularClearance) -> bool {
    let facing = crate::angle!(if reverse { core::f64::consts::PI } else { 0.0 }, rad);
    let pose = |sample: &super::PathSample| Pose::new(sample.pose.position, sample.pose.heading + facing);
    path.samples().windows(2).all(|pair| field.contains_segment(pose(&pair[0]), pose(&pair[1])))
}

/// Generate bounded cubic Hermite candidates. Their endpoint tangents are the requested
/// chassis headings (opposite for reverse travel). Reject cusps, collisions and extreme curvature.
pub fn plan(start: Pose, end: Pose, limits: Limits, field: Option<RectangularClearance>,
            direction_preference: Option<bool>) -> Option<Maneuver> {
    if !start.is_finite() || !end.is_finite() || !limits.is_valid() { return None; }
    let distance = start.position.distance(end.position);
    if distance < 0.005 { return None; }
    let mut best: Option<Maneuver> = None;
    for reverse in [false, true] {
        if direction_preference.is_some_and(|value| value != reverse) { continue; }
        let sign = if reverse { -1.0 } else { 1.0 };
        for first in [0.35, 0.65, 1.0, 1.5, 2.0] {
            for last in [0.35, 0.65, 1.0, 1.5, 2.0] {
                let a = direction(start.heading) * (sign * distance * first);
                let b = direction(end.heading) * (sign * distance * last);
                let n = ((distance * (1.0 + first + last) / 0.008).ceil() as usize).clamp(24, 240);
                let mut points = Vec::with_capacity(n + 1);
                let mut valid = true;
                for i in 0..=n {
                    let t = i as f64 / n as f64;
                    let t2 = t*t;
                    let t3 = t2*t;
                    let position = start.position*(2.0*t3-3.0*t2+1.0)
                        + a*(t3-2.0*t2+t)
                        + end.position*(-2.0*t3+3.0*t2)
                        + b*(t3-t2);
                    let derivative = (start.position-end.position)*(6.0*t2-6.0*t)
                        + a*(3.0*t2-4.0*t+1.0) + b*(3.0*t2-2.0*t);
                    if derivative.length() < distance * 0.015 { valid = false; break; }
                    points.push(Waypoint::new(position, 1.0));
                }
                if !valid { continue; }
                let Ok(path) = Path::new(&points, limits) else { continue; };
                let facing = crate::angle!(if reverse { core::f64::consts::PI } else { 0.0 }, rad);
                if wrap_heading(path.start().pose.heading + facing - start.heading).value.abs() > 0.12
                    || wrap_heading(path.end().pose.heading + facing - end.heading).value.abs() > 0.12
                    || path.samples().iter().any(|s| s.curvature.abs() * limits.track_width.value > 8.0)
                    || field.is_some_and(|f| !clear(&path, reverse, f))
                { continue; }
                if best.as_ref().is_none_or(|old| path.end().time < old.path.end().time) {
                    best = Some(Maneuver { path, reverse });
                }
            }
        }
    }
    best
}
