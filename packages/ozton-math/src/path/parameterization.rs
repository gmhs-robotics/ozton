//! Reachability-based differential wheel time parameterization.
use alloc::vec::Vec;

use super::{Limits, PathError, PathSample};

/// Linear bound a*x + b*y <= c on squared speeds at consecutive samples.
#[derive(Clone, Copy)]
struct Bound {
    a: f64,
    b: f64,
    c: f64,
}

#[derive(Clone, Copy)]
struct Line {
    slope: f64,
    intercept: f64,
}

impl Line {
    const ZERO: Self = Self {
        slope: 0.0,
        intercept: 0.0,
    };

    fn at(self, x: f64) -> f64 {
        self.slope * x + self.intercept
    }
}

struct Segment {
    bounds: [Bound; 10],
    speed_squared: f64,
}

impl Segment {
    fn new(a: PathSample, b: PathSample, limits: Limits) -> Self {
        let ds = b.distance - a.distance;
        let half = limits.track_width.value * 0.5;
        let gradient = half * (b.curvature - a.curvature) / ds;
        let rate = 0.5 / ds;
        let mut bounds = [Bound {
            a: 0.0,
            b: 0.0,
            c: 0.0,
        }; 10];
        bounds[0] = Bound {
            a: -rate,
            b: rate,
            c: limits.acceleration.value,
        };
        bounds[1] = Bound {
            a: rate,
            b: -rate,
            c: limits.deceleration.value,
        };
        let mut index = 2;
        for sign in [-1.0, 1.0] {
            for (endpoint, curvature) in [a.curvature, b.curvature].into_iter().enumerate() {
                let factor = (1.0 + sign * half * curvature) * rate;
                // a_w = (1 ± bκ)(y-x)/(2 ds) ± bκ'v².
                // Both acceleration and curvature are affine in arc length.
                let left = -factor + if endpoint == 0 { sign * gradient } else { 0.0 };
                let right = factor + if endpoint == 1 { sign * gradient } else { 0.0 };
                let c = limits.wheel_acceleration.value;
                bounds[index] = Bound {
                    a: left,
                    b: right,
                    c,
                };
                bounds[index + 1] = Bound {
                    a: -left,
                    b: -right,
                    c,
                };
                index += 2;
            }
        }
        let curvature = a.curvature.abs().max(b.curvature.abs());
        let wheel_speed = limits.wheel_speed.value / (1.0 + half * curvature);
        let speed_squared =
            (wheel_speed * wheel_speed).min(limits.lateral_acceleration.value / curvature);
        Self {
            bounds,
            speed_squared,
        }
    }

    /// Project the feasible two-variable polytope onto x. All constraints
    /// contain the origin, so the controllable set is [0, maximum].
    /// Fourier-Motzkin elimination needs no iterative solver or allocation.
    fn predecessor(&self, mut maximum: f64, next: f64) -> f64 {
        let mut lower = [Line::ZERO; 11];
        let mut upper = [Line::ZERO; 11];
        upper[0].intercept = next;
        let (mut nl, mut nu) = (1, 1);
        for bound in self.bounds {
            if bound.b == 0.0 {
                if bound.a > 0.0 {
                    maximum = maximum.min(bound.c / bound.a);
                }
                continue;
            }
            let line = Line {
                slope: -bound.a / bound.b,
                intercept: bound.c / bound.b,
            };
            if bound.b < 0.0 {
                lower[nl] = line;
                nl += 1;
            } else {
                upper[nu] = line;
                nu += 1;
            }
        }
        for l in &lower[..nl] {
            for u in &upper[..nu] {
                let slope = l.slope - u.slope;
                if slope > 0.0 {
                    maximum = maximum.min((u.intercept - l.intercept) / slope);
                }
            }
        }
        maximum.max(0.0)
    }

    /// With x fixed, take the greatest feasible y in the next controllable set.
    fn successor(&self, x: f64, mut next: f64) -> f64 {
        for bound in self.bounds {
            if bound.b > 0.0 {
                let line = Line {
                    slope: -bound.a / bound.b,
                    intercept: bound.c / bound.b,
                };
                next = next.min(line.at(x));
            }
        }
        next.max(0.0)
    }
}

/// Backward controllable-set pass followed by a forward reachable-speed pass.
/// Unlike uniform time dilation, a tight corner slows only the portions of
/// the path that must decelerate for it. Wheel acceleration is constrained
/// at both endpoints; because it is affine within each segment, this also
/// bounds the entire interpolated segment.
pub(super) fn limit_wheels(samples: &mut [PathSample], limits: Limits) -> Result<(), PathError> {
    let mut caps: Vec<_> = samples.iter().map(|p| p.speed * p.speed).collect();
    for (i, pair) in samples.windows(2).enumerate() {
        let segment = Segment::new(pair[0], pair[1], limits);
        caps[i] = caps[i].min(segment.speed_squared);
        caps[i + 1] = caps[i + 1].min(segment.speed_squared);
    }
    for i in (0..samples.len() - 1).rev() {
        let segment = Segment::new(samples[i], samples[i + 1], limits);
        caps[i] = segment.predecessor(caps[i], caps[i + 1]);
    }
    let mut x = 0.0;
    for i in 0..samples.len() - 1 {
        let segment = Segment::new(samples[i], samples[i + 1], limits);
        let y = segment.successor(x, caps[i + 1]);
        // Guard the continuous segment inequalities against arithmetic error.
        if !y.is_finite()
            || segment
                .bounds
                .iter()
                .any(|b| b.a * x + b.b * y > b.c + 1e-8 * b.c.max(1.0))
        {
            return Err(PathError::BlockedSegment(i + 1));
        }
        samples[i + 1].speed = libm::sqrt(y);
        x = y;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        geometry::DVec2,
        path::{Path, Waypoint},
        qty,
    };

    #[test]
    fn changing_curvature_respects_wheel_acceleration_between_samples() {
        let limits = Limits::from_units(
            qty!(0.33 m),
            qty!(1 m/s),
            qty!(1 m/s),
            qty!(2 m/s^2),
            qty!(2 m/s^2),
            qty!(2 m/s^2),
        )
        .with_wheel_acceleration(qty!(0.5 m/s^2));
        let points = [
            DVec2::new(0.0, 0.0),
            DVec2::new(0.0, 0.2),
            DVec2::new(0.1, 0.4),
            DVec2::new(0.3, 0.5),
            DVec2::new(0.5, 0.5),
        ]
        .map(|p| Waypoint::new(p, 1.0));
        let path = Path::new(&points, limits).unwrap();
        for pair in path.samples().windows(2) {
            let [a, b] = [pair[0], pair[1]];
            let ds = b.distance - a.distance;
            let acceleration = (b.speed * b.speed - a.speed * a.speed) / (2.0 * ds);
            let gradient = (b.curvature - a.curvature) / ds;
            for p in [a, b] {
                for sign in [-1.0, 1.0] {
                    let half = limits.track_width.value * 0.5;
                    let wheel = (1.0 + sign * half * p.curvature) * acceleration
                        + sign * half * gradient * p.speed * p.speed;
                    assert!(wheel.abs() <= limits.wheel_acceleration.value + 1e-10);
                }
            }
        }
    }
}
