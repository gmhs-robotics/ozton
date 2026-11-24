//! Continuous quintic Hermite paths with one velocity profile across all knots.

use alloc::vec::Vec;

use uom::si::f64::Length;

use super::{Limits, MAX_POINTS, Path, PathError, Waypoint};
use crate::geometry::{Pose, direction};

/// A continuous, single-direction trajectory through prescribed poses.
/// Intermediate knots share tangent and second derivative (zero).
#[derive(Debug)]
pub struct PathChain {
    knots: Vec<(Pose, f64)>,
    resolution: Length,
}

impl PathChain {
    pub fn new(start: Pose) -> Self {
        Self {
            knots: alloc::vec![(start, 1.0)],
            resolution: crate::qty!(10 mm),
        }
    }

    #[must_use]
    pub fn then(self, pose: Pose) -> Self {
        self.then_with_speed(pose, 1.0)
    }

    #[must_use]
    pub fn then_with_speed(mut self, pose: Pose, speed_fraction: f64) -> Self {
        self.knots.push((pose, speed_fraction));
        self
    }

    #[must_use]
    pub const fn with_resolution(mut self, resolution: Length) -> Self {
        self.resolution = resolution;
        self
    }

    pub fn build(self, limits: Limits) -> Result<Path, PathError> {
        if self.knots.len() < 2 {
            return Err(PathError::TooFewPoints);
        }
        if !self.resolution.value.is_finite() || self.resolution.value <= 0.0 {
            return Err(PathError::InvalidLimits);
        }
        let lengths: Vec<_> = self
            .knots
            .windows(2)
            .map(|pair| pair[0].0.position.distance(pair[1].0.position))
            .collect();
        for (i, &(pose, speed)) in self.knots.iter().enumerate() {
            if !pose.is_finite()
                || !(0.0..=1.0).contains(&speed)
                || lengths.get(i).is_some_and(|length| *length <= 1e-6)
            {
                return Err(PathError::InvalidPoint(i));
            }
        }
        let tangents: Vec<_> = self
            .knots
            .iter()
            .enumerate()
            .map(|(i, (pose, _))| {
                let before = lengths[i.saturating_sub(1)];
                let after = lengths[i.min(lengths.len() - 1)];
                direction(pose.heading) * before.min(after)
            })
            .collect();
        let mut points = Vec::new();
        for (i, pair) in self.knots.windows(2).enumerate() {
            let [(a, start_speed), (b, end_speed)] = [pair[0], pair[1]];
            let delta = b.position - a.position;
            let (d0, d1) = (tangents[i], tangents[i + 1]);
            let segments =
                libm::ceil((lengths[i] + d0.length() + d1.length()) / self.resolution.value)
                    .max(2.0) as usize;
            if segments > MAX_POINTS.saturating_sub(points.len() + 1) {
                return Err(PathError::TooManyPoints);
            }
            for j in 0..segments {
                let t = j as f64 / segments as f64;
                let t2 = t * t;
                let t3 = t2 * t;
                let t4 = t3 * t;
                let t5 = t4 * t;
                let position = a.position
                    + delta * (10.0 * t3 - 15.0 * t4 + 6.0 * t5)
                    + d0 * (t - 6.0 * t3 + 8.0 * t4 - 3.0 * t5)
                    + d1 * (-4.0 * t3 + 7.0 * t4 - 3.0 * t5);
                let derivative = delta * (30.0 * t2 - 60.0 * t3 + 30.0 * t4)
                    + d0 * (1.0 - 18.0 * t2 + 32.0 * t3 - 15.0 * t4)
                    + d1 * (-12.0 * t2 + 28.0 * t3 - 15.0 * t4);
                if derivative.length_squared() <= 1e-12 {
                    return Err(PathError::InvalidPoint(i));
                }
                points.push(Waypoint::new(
                    position,
                    start_speed + (end_speed - start_speed) * t,
                ));
            }
        }
        let (end, speed) = self.knots[self.knots.len() - 1];
        points.push(Waypoint::new(end.position, speed));
        Path::new(&points, limits)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{position_xyh, qty};

    #[test]
    fn chain_passes_middle_knot_with_nonzero_speed() {
        let limits = Limits::from_units(
            qty!(13 in),
            qty!(1 m/s),
            qty!(1 m/s),
            qty!(1 m/s^2),
            qty!(1 m/s^2),
            qty!(1 m/s^2),
        );
        let middle = position_xyh!(1 in, 12 in, 0 deg);
        let path = PathChain::new(Pose::default())
            .then(middle)
            .then(position_xyh!(0 in, 24 in, 0 deg))
            .build(limits)
            .unwrap();
        let knot = path
            .samples()
            .iter()
            .min_by(|a, b| {
                a.pose
                    .position
                    .distance_squared(middle.position)
                    .total_cmp(&b.pose.position.distance_squared(middle.position))
            })
            .unwrap();
        assert!(knot.pose.position.distance(middle.position) < 1e-9);
        assert!(knot.speed > 0.1);
        assert_eq!(path.start().speed, 0.0);
        assert_eq!(path.end().speed, 0.0);
    }
}
