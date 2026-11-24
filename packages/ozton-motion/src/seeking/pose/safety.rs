//! Predictive stopping envelope and curvature-preserving command governor.
use ozton_localization::Feedback;
use ozton_math::{geometry::DVec2, path::clearance::RectangularClearance, qty};
use ozton_runtime::{log_event, log_row};

use crate::{MotionConfig, basic::differential::DriveOptions};

/// Constant-speed reaction interval followed by bounded constant deceleration.
fn stop(v: f64, deceleration: f64, reaction: f64) -> f64 {
    v * reaction + v * v.abs() / (2.0 * deceleration)
}

fn interval(v: f64, sigma: f64, deceleration: f64, reaction: f64) -> [f64; 2] {
    [
        stop(v.min(0.0) - sigma, deceleration, reaction),
        stop(v.max(0.0) + sigma, deceleration, reaction),
    ]
}

fn bounds(
    field: RectangularClearance,
    feedback: Feedback,
    v: f64,
    w: f64,
    uncertainty: [f64; 3],
    reaction: f64,
    config: MotionConfig,
) -> Option<(DVec2, DVec2)> {
    if !uncertainty.into_iter().all(|s| s.is_finite() && s >= 0.0) {
        return None;
    }
    let [lateral, forward, yaw] = uncertainty.map(|s| s * config.braking_sigma);
    let forward = interval(
        v,
        forward,
        config.forward_braking.unwrap_or(config.deceleration).value,
        reaction,
    );
    let lateral = interval(
        feedback.lateral_velocity.value,
        lateral,
        config.lateral_braking.value,
        reaction,
    );
    let yaw = stop(
        w.abs() + yaw,
        config
            .yaw_braking
            .unwrap_or(config.angular_deceleration)
            .value,
        reaction,
    );
    field.braking_bounds(feedback.pose, forward, lateral, qty!((yaw) rad))
}

fn reaction(config: MotionConfig) -> f64 {
    config.maximum_gap.as_secs_f64() + config.brake_latency.as_secs_f64()
}

/// Maximize wheel scaling subject to the predicted swept-footprint constraints.
/// Because each signed displacement interval expands monotonically with
/// command magnitude, bisection finds a feasible scale without changing curvature.
pub(super) fn limit_wall_command(
    field: Option<RectangularClearance>,
    feedback: Feedback,
    wheels: [f64; 2],
    options: DriveOptions,
    config: MotionConfig,
    uncertainty: [f64; 3],
) -> [f64; 2] {
    let Some(field) = field else {
        return wheels;
    };
    let v = (wheels[0] + wheels[1]) * 0.5;
    let w = (wheels[1] - wheels[0]) / options.track_width.value;
    let delay = reaction(config) + config.wall_preview.as_secs_f64();
    let clear = |scale| {
        bounds(
            field,
            feedback,
            v * scale,
            w * scale,
            uncertainty,
            delay,
            config,
        )
        .is_some_and(|(min, max)| field.contains_aabb(min, max))
    };
    if clear(1.0) {
        return wheels;
    }
    let mut low = 0.0;
    let mut high = 1.0;
    for _ in 0..12 {
        let middle = (low + high) * 0.5;
        if clear(middle) {
            low = middle;
        } else {
            high = middle;
        }
    }
    wheels.map(|speed| speed * low)
}

pub(super) fn safe_feedback(
    field: RectangularClearance,
    feedback: Feedback,
    config: MotionConfig,
    uncertainty: [f64; 3],
) -> bool {
    let v = feedback.forward_velocity.value;
    let w = feedback.angular_velocity.value;
    let projected = bounds(field, feedback, v, w, uncertainty, reaction(config), config);
    let reason = if !field.contains_pose(feedback.pose) {
        "current_footprint"
    } else if !projected.is_some_and(|(min, max)| field.wall_contains_aabb(min, max)) {
        "wall_stopping_envelope"
    } else if !projected.is_some_and(|(min, max)| field.contains_aabb(min, max)) {
        "obstacle_stopping_envelope"
    } else {
        return true;
    };
    let explanation = match reason {
        "current_footprint" => "Current chassis footprint violates clearance",
        "wall_stopping_envelope" => "Predicted braking sweep exceeds the wall boundary",
        _ => "Predicted braking sweep intersects a hard obstacle",
    };
    log_event!(Warn, "clearance", "{explanation}; motion rejected");
    let (min, max) = projected.unwrap_or((DVec2::splat(f64::NAN), DVec2::splat(f64::NAN)));
    log_row!(
        Warn,
        "CLEARANCE_FAULT",
        "reason={reason},x_m={:.5},y_m={:.5},heading_rad={:.5},forward_mps={v:.5},yaw_radps={w:.5},min_x_m={:.5},min_y_m={:.5},max_x_m={:.5},max_y_m={:.5}",
        feedback.pose.position.x,
        feedback.pose.position.y,
        feedback.pose.heading.value,
        min.x,
        min.y,
        max.x,
        max.y
    );
    false
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use ozton_math::{geometry::Pose, path::clearance::Footprint};

    use super::*;

    fn sample(v: f64) -> Feedback {
        Feedback {
            time: Duration::ZERO,
            pose: Pose::new(DVec2::new(0.20411, -0.01730), qty!(-1.54347 rad)),
            forward_travel: qty!(0 m),
            forward_velocity: qty!((v) m/s),
            lateral_velocity: qty!(0 m/s),
            angular_velocity: qty!(0.10613 rad/s),
            ranges: [None; _],
            reset_accepted: false,
        }
    }

    fn field() -> RectangularClearance {
        RectangularClearance::new(
            Footprint::new(qty!(15.1 in), qty!(15 in)).unwrap(),
            DVec2::new(0.93345, 1.3843),
            qty!(1 in),
        )
        .unwrap()
    }

    #[test]
    fn shared_velocity_and_uncertainty_govern_wall_approach() {
        let field = field();
        let config = MotionConfig {
            deceleration: qty!(0.55 m/s^2),
            ..MotionConfig::DEFAULT
        };
        assert!(!safe_feedback(field, sample(0.88620), config, [0.0; 3]));
        assert!(safe_feedback(field, sample(0.549), config, [0.0; 3]));
        assert!(!safe_feedback(
            field,
            sample(0.549),
            config,
            [0.0, 0.2, 0.0]
        ));
        let options = DriveOptions::differential(qty!(0.3292677692 m), qty!(0.55 m/s));
        let mut near = sample(0.549);
        near.pose.position.x = 0.55;
        let limited = limit_wall_command(Some(field), near, [0.55; 2], options, config, [0.0; 3]);
        assert!(limited[0] > 0.0 && limited[0] < 0.55);
        assert_eq!(limited[0], limited[1]);
    }

    #[test]
    fn logged_diagonal_stall_allows_a_governed_pivot() {
        let field = RectangularClearance::new(
            Footprint::from_extents(
                qty!(7.55 in),
                qty!(7.55 in),
                qty!(7.761804 in),
                qty!(7.238196 in),
            )
            .unwrap(),
            DVec2::new(0.93345, 1.3843),
            qty!(0.5 in),
        )
        .unwrap();
        let mut feedback = sample(0.00011);
        feedback.pose = Pose::new(DVec2::new(0.48862, 0.01052), qty!(-0.89179 rad));
        feedback.angular_velocity = qty!(0 rad/s);
        let config =
            MotionConfig::DEFAULT.with_braking_deceleration(qty!(0.55 m/s^2), qty!(3 rad/s^2));
        let uncertainty = [0.1117, 0.0607, 0.0474];
        assert!(safe_feedback(field, feedback, config, uncertainty));
        let options = DriveOptions::differential(qty!(0.3292677692 m), qty!(0.60 m/s));
        let wheels = limit_wall_command(
            Some(field),
            feedback,
            [-0.2, 0.2],
            options,
            config,
            uncertainty,
        );
        assert!(wheels[1] > 0.04 && wheels[1] < 0.2);
        assert_eq!(wheels[0], -wheels[1]);
        // The same request near the physical boundary must still be rejected.
        feedback.pose.position.x = 0.85;
        assert!(!safe_feedback(field, feedback, config, uncertainty));
        assert_eq!(
            limit_wall_command(
                Some(field),
                feedback,
                [-0.2, 0.2],
                options,
                config,
                uncertainty,
            ),
            [0.0; 2]
        );
    }

    #[test]
    fn diagonal_lateral_and_angular_sweep_cannot_cancel() {
        let f = field();
        let pose = Pose::new(DVec2::new(0.5, 0.0), qty!(-45 deg));
        let (min, max) = f
            .braking_bounds(pose, [0.0, 0.4], [-0.4, 0.0], qty!(30 deg))
            .unwrap();
        assert!(!f.wall_contains_aabb(min, max));
    }
}
