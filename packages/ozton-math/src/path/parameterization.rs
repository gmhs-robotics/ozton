//! Differential wheel constraints for a piecewise-linear curvature profile.
use super::{Limits, PathSample};

/// A conservative uniform time dilation of the existing forward/backward
/// profile. For each segment, v² and curvature are affine in arc length.
/// Wheel acceleration a_w = (1 ± bκ)a ± bκ'v² is therefore affine too;
/// its maximum magnitude occurs at an endpoint. Scaling v by λ scales
/// acceleration by λ², so a single pass enforces the bound everywhere.
pub(super) fn limit_wheels(samples: &mut [PathSample], limits: Limits) {
    let half_track = limits.track_width.value * 0.5;
    let mut scale: f64 = 1.0;
    for pair in samples.windows(2) {
        let [a, b] = [pair[0], pair[1]];
        let ds = b.distance - a.distance;
        let v2 = [a.speed * a.speed, b.speed * b.speed];
        let acceleration = (v2[1] - v2[0]) / (2.0 * ds);
        let gradient = half_track * (b.curvature - a.curvature) / ds;
        for sign in [-1.0, 1.0] {
            let factors = [
                1.0 + sign * half_track * a.curvature,
                1.0 + sign * half_track * b.curvature,
            ];
            let peak_acceleration = (factors[0] * acceleration + sign * gradient * v2[0])
                .abs()
                .max((factors[1] * acceleration + sign * gradient * v2[1]).abs());
            if peak_acceleration > limits.wheel_acceleration.value {
                scale = scale.min(libm::sqrt(
                    limits.wheel_acceleration.value / peak_acceleration,
                ));
            }
            // Product bound also covers an interior wheel-speed maximum.
            let peak_speed = a.speed.max(b.speed) * factors[0].abs().max(factors[1].abs());
            if peak_speed > limits.wheel_speed.value {
                scale = scale.min(limits.wheel_speed.value / peak_speed);
            }
        }
    }
    for sample in samples {
        sample.speed *= scale;
    }
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
