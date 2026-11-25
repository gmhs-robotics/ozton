//! Bounded, clearance-aware differential pose planning in a rectangular field.
use alloc::vec::Vec;

mod lattice;
pub use lattice::{LatticePlanner, LatticeStep};

pub use super::clearance::AStar;
use super::{Limits, Path, Waypoint, clearance::RectangularClearance};
use crate::geometry::{Pose, direction, wrap_heading};

/// A forward or backward trajectory with the lowest predicted time among tested candidates.
pub struct Maneuver {
    pub path: Path,
    pub reverse: bool,
}

/// Validate the oriented footprint over each interpolated segment, including reverse travel.
pub fn clear(path: &Path, reverse: bool, field: RectangularClearance) -> bool {
    let facing = crate::qty!((if reverse { core::f64::consts::PI } else { 0.0 }) rad);
    let pose =
        |sample: &super::PathSample| Pose::new(sample.pose.position, sample.pose.heading + facing);
    path.samples()
        .windows(2)
        .all(|pair| field.contains_segment(pose(&pair[0]), pose(&pair[1])))
}

/// Check the swept footprint cooperatively, with a cancellation checkpoint
/// between segments. The synchronous variant remains available to host tools.
pub async fn clear_cooperative<F>(
    path: &Path,
    reverse: bool,
    field: RectangularClearance,
    mut checkpoint: F,
) -> bool
where
    F: core::ops::AsyncFnMut() -> bool,
{
    let facing = crate::qty!((if reverse { core::f64::consts::PI } else { 0.0 }) rad);
    let pose =
        |sample: &super::PathSample| Pose::new(sample.pose.position, sample.pose.heading + facing);
    for pair in path.samples().windows(2) {
        if !checkpoint().await || !field.contains_segment(pose(&pair[0]), pose(&pair[1])) {
            return false;
        }
    }
    true
}

/// Incremental cubic Hermite search. Each step evaluates one candidate so a
/// cooperative runtime can service tracking and other time-sensitive tasks.
pub struct ManeuverSearch {
    start: Pose,
    end: Pose,
    limits: Limits,
    field: Option<RectangularClearance>,
    direction_preference: Option<bool>,
    distance: f64,
    start_direction: crate::geometry::DVec2,
    end_direction: crate::geometry::DVec2,
    minimum_derivative_squared: f64,
    points: Vec<Waypoint>,
    best: Option<Maneuver>,
    next: usize,
}

impl ManeuverSearch {
    pub fn new(
        start: Pose,
        end: Pose,
        limits: Limits,
        field: Option<RectangularClearance>,
        direction_preference: Option<bool>,
    ) -> Option<Self> {
        if !start.is_finite() || !end.is_finite() || !limits.is_valid() {
            return None;
        }
        let distance = start.position.distance(end.position);
        if distance < 0.005 {
            return None;
        }
        let minimum_derivative = distance * 0.015;
        Some(Self {
            start,
            end,
            limits,
            field,
            direction_preference,
            distance,
            start_direction: direction(start.heading) * distance,
            end_direction: direction(end.heading) * distance,
            minimum_derivative_squared: minimum_derivative * minimum_derivative,
            points: Vec::new(),
            best: None,
            next: 0,
        })
    }

    /// Evaluate at most one candidate. Returns false when the search is exhausted.
    pub fn step(&mut self) -> bool {
        const TANGENTS: [f64; 5] = [0.35, 0.65, 1.0, 1.5, 2.0];
        while self.next < 50 {
            let index = self.next;
            self.next += 1;
            let reverse = index >= 25;
            if self
                .direction_preference
                .is_some_and(|value| value != reverse)
            {
                continue;
            }
            self.consider(reverse, TANGENTS[(index / 5) % 5], TANGENTS[index % 5]);
            return true;
        }
        false
    }

    /// Return the best valid candidate considered so far.
    pub fn finish(self) -> Option<Maneuver> {
        self.best
    }

    fn consider(&mut self, reverse: bool, first: f64, last: f64) {
        let sign = if reverse { -1.0 } else { 1.0 };
        let facing = crate::qty!((if reverse { core::f64::consts::PI } else { 0.0 }) rad);
        let a = self.start_direction * (sign * first);
        let b = self.end_direction * (sign * last);
        let n = (libm::ceil(self.distance * (1.0 + first + last) / 0.008) as usize).clamp(24, 240);
        self.points.clear();
        self.points.reserve(n + 1);
        let inverse_n = 1.0 / n as f64;
        for i in 0..=n {
            let t = i as f64 * inverse_n;
            let t2 = t * t;
            let t3 = t2 * t;
            let position = self.start.position * (2.0 * t3 - 3.0 * t2 + 1.0)
                + a * (t3 - 2.0 * t2 + t)
                + self.end.position * (-2.0 * t3 + 3.0 * t2)
                + b * (t3 - t2);
            let derivative = (self.start.position - self.end.position) * (6.0 * t2 - 6.0 * t)
                + a * (3.0 * t2 - 4.0 * t + 1.0)
                + b * (3.0 * t2 - 2.0 * t);
            if derivative.length_squared() < self.minimum_derivative_squared {
                return;
            }
            self.points.push(Waypoint::new(position, 1.0));
        }
        let Ok(path) = Path::new(&self.points, self.limits) else {
            return;
        };
        if self
            .best
            .as_ref()
            .is_some_and(|old| path.end().time >= old.path.end().time)
        {
            return;
        }
        if wrap_heading(path.start().pose.heading + facing - self.start.heading)
            .value
            .abs()
            > 0.12
            || wrap_heading(path.end().pose.heading + facing - self.end.heading)
                .value
                .abs()
                > 0.12
            || path
                .samples()
                .iter()
                .any(|s| s.curvature.abs() * self.limits.track_width.value > 8.0)
            || self.field.is_some_and(|f| !clear(&path, reverse, f))
        {
            return;
        }
        self.best = Some(Maneuver { path, reverse });
    }
}

/// Generate bounded cubic Hermite candidates. Their endpoint tangents are the requested
/// chassis headings (opposite for reverse travel). Reject cusps, collisions and extreme curvature.
pub fn plan(
    start: Pose,
    end: Pose,
    limits: Limits,
    field: Option<RectangularClearance>,
    direction_preference: Option<bool>,
) -> Option<Maneuver> {
    let mut search = ManeuverSearch::new(start, end, limits, field, direction_preference)?;
    while search.step() {}
    search.finish()
}
