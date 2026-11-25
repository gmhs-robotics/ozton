/// Bound lateral acceleration of a differential wheel command while preserving
/// curvature. The same bound is needed after independent linear/yaw slew.
pub(super) fn limit_lateral_acceleration(
    wheels: [f64; 2],
    track_width: f64,
    maximum: f64,
) -> [f64; 2] {
    let linear = (wheels[0] + wheels[1]) * 0.5;
    let angular = (wheels[1] - wheels[0]) / track_width;
    let lateral = (linear * angular).abs();
    if lateral <= maximum {
        return wheels;
    }
    let scale = (maximum / lateral).sqrt();
    wheels.map(|speed| speed * scale)
}

/// Limit the growing component of a slew-limited command rather than
/// abruptly scaling both components. If the previous and desired twists
/// are feasible, at most one component can grow into an infeasible region.
/// Reducing its change preserves the linear, yaw and wheel slew bounds.
pub(super) fn limit_transition(
    previous: [f64; 2],
    candidate: [f64; 2],
    width: f64,
    wheel_maximum: f64,
    lateral_maximum: f64,
) -> [f64; 2] {
    let previous_v = (previous[0] + previous[1]) * 0.5;
    let mut v = (candidate[0] + candidate[1]) * 0.5;
    let mut w = (candidate[1] - candidate[0]) / width;
    let half = width * 0.5;
    if v.abs() > previous_v.abs() {
        let maximum = (wheel_maximum - w.abs() * half)
            .max(0.0)
            .min(lateral_maximum / w.abs());
        v = v.clamp(-maximum, maximum);
    } else {
        let maximum = ((wheel_maximum - v.abs()) / half)
            .max(0.0)
            .min(lateral_maximum / v.abs());
        w = w.clamp(-maximum, maximum);
    }
    [v - w * half, v + w * half]
}

#[cfg(test)]
mod tests {
    use super::{limit_lateral_acceleration, limit_transition};

    #[test]
    fn lateral_limit_preserves_curvature_and_turn_direction() {
        let width = 0.33;
        let maximum = 0.55;
        for wheels in [[0.4, 1.4], [1.4, 0.4], [-0.4, -1.4], [-1.4, -0.4]] {
            let limited = limit_lateral_acceleration(wheels, width, maximum);
            let v = (limited[0] + limited[1]) * 0.5;
            let w = (limited[1] - limited[0]) / width;
            assert!(((v * w).abs() - maximum).abs() < 1e-12);
            assert!((limited[0] * wheels[1] - limited[1] * wheels[0]).abs() < 1e-12);
        }
    }

    #[test]
    fn turning_transition_respects_traction_without_exceeding_slew() {
        let width = 0.33;
        let maximum = 0.55;
        let wheels = |v, w| [v - w * width * 0.5, v + w * width * 0.5];
        for (previous, candidate) in [
            (wheels(1.0, 0.5), wheels(0.99, 0.8)),
            (wheels(0.5, 1.0), wheels(0.8, 0.99)),
            (wheels(-1.0, -0.5), wheels(-0.99, -0.8)),
        ] {
            let applied = limit_transition(previous, candidate, width, 1.5, maximum);
            let v = (applied[0] + applied[1]) * 0.5;
            let w = (applied[1] - applied[0]) / width;
            assert!((v * w).abs() <= maximum + 1e-12);
            let change = |pair: [f64; 2]| {
                (pair[0] - previous[0])
                    .abs()
                    .max((pair[1] - previous[1]).abs())
            };
            assert!(change(applied) <= change(candidate) + 1e-12);
        }
    }

    #[test]
    fn lateral_limit_leaves_straight_and_stationary_turn_commands_unchanged() {
        for wheels in [[1.0, 1.0], [-1.0, -1.0], [-0.5, 0.5], [0.0, 0.0]] {
            assert_eq!(limit_lateral_acceleration(wheels, 0.33, 0.55), wheels);
        }
    }
}
