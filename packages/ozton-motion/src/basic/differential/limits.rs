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

#[cfg(test)]
mod tests {
    use super::limit_lateral_acceleration;

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
    fn lateral_limit_leaves_straight_and_stationary_turn_commands_unchanged() {
        for wheels in [[1.0, 1.0], [-1.0, -1.0], [-0.5, 0.5], [0.0, 0.0]] {
            assert_eq!(limit_lateral_acceleration(wheels, 0.33, 0.55), wheels);
        }
    }
}
