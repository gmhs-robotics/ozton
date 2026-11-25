//! Standard unicycle feedback laws, with forward/left/CCW error convention.

use crate::PathGuidance;

/// RAMSETE (nonlinear) and continuous-time gain-scheduled LQR. The LQR uses
/// the straight-reference linearization: e_y' = v e_theta - delta v_y,
/// e_theta' = -delta omega. The differential drive has no delta v_y input.
/// The scalar and 2x2 algebraic Riccati equations have closed-form solutions.
pub(super) fn track(guidance: PathGuidance, error: [f64; 3], reference: [f64; 2]) -> [f64; 2] {
    let [x, y, theta] = error;
    let [v, w] = reference;
    match guidance {
        PathGuidance::Ramsete { b, zeta } => {
            let gain = 2.0 * zeta * (w * w + b * v * v).sqrt();
            let sinc = if theta.abs() < 1e-6 {
                1.0 - theta * theta / 6.0
            } else {
                theta.sin() / theta
            };
            [
                v * theta.cos() + gain * x,
                w + gain * theta + b * v * sinc * y,
            ]
        }
        PathGuidance::Lqr { state, input } => {
            let [qx, qy, qt] = state;
            let [rv, rw] = input;
            let kx = (qx / rv).sqrt();
            let ky = (qy / rw).sqrt();
            let kt = (qt / rw + 2.0 * v.abs() * ky).sqrt();
            [v + kx * x, w + v.signum() * ky * y + kt * theta]
        }
        _ => reference,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_reference_is_unchanged() {
        for mode in [PathGuidance::RAMSETE, PathGuidance::LQR] {
            for reference in [[0.8, 1.2], [-0.8, 1.2], [0.0, 0.0]] {
                assert_eq!(track(mode, [0.0; 3], reference), reference);
            }
        }
    }

    #[test]
    fn lateral_feedback_reverses_with_travel_direction() {
        for mode in [PathGuidance::RAMSETE, PathGuidance::LQR] {
            assert!(track(mode, [0.0, 0.05, 0.0], [0.6, 0.0])[1] > 0.0);
            assert!(track(mode, [0.0, 0.05, 0.0], [-0.6, 0.0])[1] < 0.0);
        }
    }

    #[test]
    fn longitudinal_and_heading_errors_have_restoring_sign() {
        for mode in [PathGuidance::RAMSETE, PathGuidance::LQR] {
            let command = track(mode, [0.1, 0.0, 0.1], [0.5, 0.0]);
            assert!(command[0] > 0.5 && command[1] > 0.0);
        }
    }
}
