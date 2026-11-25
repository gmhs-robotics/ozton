//! Predictive stopping envelope and curvature-preserving command governor.
use std::cell::Cell;

use ozton_diagnostics::{log_event, log_row};
use ozton_localization::Feedback;
use ozton_math::{
    geometry::DVec2,
    path::{
        braking::{RateBounds, position_padding},
        clearance::RectangularClearance,
    },
    qty,
    uom::si::f64::Velocity,
};

use crate::{MotionConfig, basic::differential::DriveOptions};

/// Shared stopping-envelope governor for closed-loop and raw wheel commands.
pub struct StoppingGovernor {
    field: Option<RectangularClearance>,
    options: DriveOptions,
    config: MotionConfig,
    previous: Cell<[f64; 2]>,
}

impl StoppingGovernor {
    pub(super) fn new(
        field: Option<RectangularClearance>,
        options: DriveOptions,
        config: MotionConfig,
    ) -> Self {
        Self {
            field,
            options,
            config,
            previous: Cell::new([0.0; 2]),
        }
    }

    /// Returns the admissible command, or None if the current measured
    /// motion is already outside the stopping envelope.
    pub fn govern(
        &self,
        feedback: Feedback,
        uncertainty: [f64; 3],
        covariance: Option<glam::DMat2>,
        requested: [Velocity; 2],
    ) -> Option<[Velocity; 2]> {
        let previous = self.previous.get();
        if requested.iter().any(|v| !v.value.is_finite())
            || !feedback.pose.is_finite()
            || !feedback.forward_velocity.value.is_finite()
            || !feedback.lateral_velocity.value.is_finite()
            || !feedback.angular_velocity.value.is_finite()
            || self.field.is_some_and(|field| {
                !safe_feedback(
                    field,
                    feedback,
                    self.config,
                    uncertainty,
                    covariance,
                    body_rates(previous, self.options.track_width.value),
                )
            })
        {
            return None;
        }
        let limited = limit_wall_command(
            self.field,
            feedback,
            requested.map(|v| v.value),
            self.options,
            self.config,
            uncertainty,
            covariance,
            previous,
        );
        self.previous.set(limited);
        Some(limited.map(|value| qty!((value) m/s)))
    }
}

pub(super) fn body_rates([left, right]: [f64; 2], track: f64) -> [f64; 2] {
    [(left + right) * 0.5, (right - left) / track]
}

fn bounds(
    field: RectangularClearance,
    feedback: Feedback,
    v: f64,
    w: f64,
    previous: [f64; 2],
    uncertainty: [f64; 3],
    covariance: Option<glam::DMat2>,
    reaction: f64,
    config: MotionConfig,
) -> Option<(DVec2, DVec2)> {
    let [lateral, forward, yaw] = uncertainty.map(|s| s * config.safety.braking_sigma);
    let forward = RateBounds {
        measured: feedback.forward_velocity.value,
        requested: v,
        previous: previous[0],
        uncertainty: forward,
        acceleration: config.safety.forward_acceleration_bound.map(|a| a.value),
        braking: config
            .safety
            .forward_braking
            .unwrap_or(config.limits.deceleration)
            .value,
    }
    .stopping_interval(reaction)?;
    let lateral = RateBounds {
        measured: feedback.lateral_velocity.value,
        requested: feedback.lateral_velocity.value,
        previous: feedback.lateral_velocity.value,
        uncertainty: lateral,
        acceleration: None,
        braking: config.safety.lateral_braking.value,
    }
    .stopping_interval(reaction)?;
    let yaw = RateBounds {
        measured: feedback.angular_velocity.value,
        requested: w,
        previous: previous[1],
        uncertainty: yaw,
        acceleration: config.safety.yaw_acceleration_bound.map(|a| a.value),
        braking: config
            .safety
            .yaw_braking
            .unwrap_or(config.limits.angular_deceleration)
            .value,
    }
    .stopping_interval(reaction)?;
    let (min, max) =
        field.braking_bounds(feedback.pose, forward, lateral, yaw.map(|a| qty!((a) rad)))?;
    let padding = position_padding(covariance, config.safety.braking_sigma)?;
    Some((min - padding, max + padding))
}

fn reaction(config: MotionConfig) -> f64 {
    config.timing.maximum_gap.as_secs_f64() + config.safety.brake_latency.as_secs_f64()
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
    covariance: Option<glam::DMat2>,
    previous: [f64; 2],
) -> [f64; 2] {
    let Some(field) = field else {
        return wheels;
    };
    let [v, w] = body_rates(wheels, options.track_width.value);
    let previous = body_rates(previous, options.track_width.value);
    let delay = reaction(config) + config.safety.wall_preview.as_secs_f64();
    let projected = |scale| {
        bounds(
            field,
            feedback,
            v * scale,
            w * scale,
            previous,
            uncertainty,
            covariance,
            delay,
            config,
        )
    };
    let full = projected(1.0);
    if full.is_some_and(|(min, max)| field.contains_aabb(min, max)) {
        return wheels;
    }
    let clear = |scale| projected(scale).is_some_and(|(min, max)| field.contains_aabb(min, max));
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
    let (min, max) = full.unwrap_or((DVec2::splat(f64::NAN), DVec2::splat(f64::NAN)));
    let reason = if full.is_none() {
        "invalid_envelope"
    } else if !field.wall_contains_aabb(min, max) {
        "wall"
    } else {
        "obstacle"
    };
    log_row!(
        Trace,
        "GOVERNOR_LIMIT",
        "reason={reason},sample_us={},scale={low:.5},requested_left_mps={:.5},requested_right_mps={:.5},prior_v_mps={:.5},prior_yaw_radps={:.5},sigma_lateral={:.5},sigma_forward={:.5},sigma_yaw={:.5},min_x_m={:.5},min_y_m={:.5},max_x_m={:.5},max_y_m={:.5}",
        feedback.time.as_micros(),
        wheels[0],
        wheels[1],
        previous[0],
        previous[1],
        uncertainty[0],
        uncertainty[1],
        uncertainty[2],
        min.x,
        min.y,
        max.x,
        max.y
    );
    wheels.map(|speed| speed * low)
}

pub(super) fn safe_feedback(
    field: RectangularClearance,
    feedback: Feedback,
    config: MotionConfig,
    uncertainty: [f64; 3],
    covariance: Option<glam::DMat2>,
    previous: [f64; 2],
) -> bool {
    let v = feedback.forward_velocity.value;
    let w = feedback.angular_velocity.value;
    let projected = bounds(
        field,
        feedback,
        v,
        w,
        previous,
        uncertainty,
        covariance,
        reaction(config),
        config,
    );
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
        let config = MotionConfig::DEFAULT
            .with_limits(crate::MotionLimits::DEFAULT.with_deceleration(qty!(0.55 m/s^2)));
        assert!(!safe_feedback(
            field,
            sample(0.88620),
            config,
            [0.0; 3],
            None,
            [0.0; 2]
        ));
        assert!(safe_feedback(
            field,
            sample(0.549),
            config,
            [0.0; 3],
            None,
            [0.0; 2]
        ));
        assert!(!safe_feedback(
            field,
            sample(0.549),
            config,
            [0.0, 0.2, 0.0],
            None,
            [0.0; 2],
        ));
        let options = DriveOptions::differential(qty!(0.3292677692 m), qty!(0.55 m/s));
        let mut near = sample(0.10);
        near.pose.position.x = 0.55;
        let limited = limit_wall_command(
            Some(field),
            near,
            [0.55; 2],
            options,
            config,
            [0.0; 3],
            None,
            [0.0; 2],
        );
        assert!(limited[0] > 0.0 && limited[0] < 0.55);
        assert_eq!(limited[0], limited[1]);
    }

    #[test]
    fn diagonal_pivot_respects_available_clearance() {
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
        let config = MotionConfig::DEFAULT.with_safety(
            crate::ClearanceConfig::DEFAULT
                .with_forward_braking(qty!(0.55 m/s^2))
                .with_yaw_braking(qty!(3 rad/s^2)),
        );
        let uncertainty = [0.1117, 0.0607, 0.0474];
        assert!(safe_feedback(
            field,
            feedback,
            config,
            uncertainty,
            None,
            [0.0; 2]
        ));
        let options = DriveOptions::differential(qty!(0.3292677692 m), qty!(0.60 m/s));
        let wheels = limit_wall_command(
            Some(field),
            feedback,
            [-0.2, 0.2],
            options,
            config,
            uncertainty,
            None,
            [0.0; 2],
        );
        assert!(wheels[1] > 0.04 && wheels[1] < 0.2);
        assert_eq!(wheels[0], -wheels[1]);
        // The same request near the physical boundary must still be rejected.
        feedback.pose.position.x = 0.85;
        assert!(!safe_feedback(
            field,
            feedback,
            config,
            uncertainty,
            None,
            [0.0; 2]
        ));
        assert_eq!(
            limit_wall_command(
                Some(field),
                feedback,
                [-0.2, 0.2],
                options,
                config,
                uncertainty,
                None,
                [0.0; 2],
            ),
            [0.0; 2]
        );
    }

    #[test]
    fn surveyed_wall_gap_is_required_for_uncertain_departure() {
        let half_extents = DVec2::new(1.1557, 1.2319);
        let footprint = Footprint::new(qty!(15.1 in), qty!(15 in)).unwrap();
        let field = RectangularClearance::new(footprint, half_extents, qty!(0 in)).unwrap();
        let config = MotionConfig::DEFAULT;
        let options = DriveOptions::differential(qty!(0.3292677692 m), qty!(0.55 m/s));
        let mut feedback = sample(0.0);
        feedback.angular_velocity = qty!(0 rad/s);
        feedback.pose = Pose::new(
            DVec2::new(0.0, -half_extents.y + qty!(7.5 in).value),
            qty!(0 rad),
        );
        let covariance = Some(glam::DMat2::from_diagonal(DVec2::splat(0.0001)));
        let uncertainty = [0.01; 3];
        let requested = [qty!(0.20 m/s); 2];

        // A nominal contact pose does not certify containment with nonzero
        // localization uncertainty, even for a command pointing into the field.
        assert!(
            StoppingGovernor::new(Some(field), options, config)
                .govern(feedback, uncertainty, covariance, requested)
                .is_none()
        );

        // A surveyed inset lets the normal governor certify the same departure.
        feedback.pose.position.y += qty!(6 in).value;
        let allowed = StoppingGovernor::new(Some(field), options, config)
            .govern(feedback, uncertainty, covariance, requested)
            .unwrap();
        assert_eq!(allowed, requested);
    }

    #[test]
    fn diagonal_lateral_and_angular_sweep_cannot_cancel() {
        let f = field();
        let pose = Pose::new(DVec2::new(0.5, 0.0), qty!(-45 deg));
        let (min, max) = f
            .braking_bounds(pose, [0.0, 0.4], [-0.4, 0.0], [qty!(-30 deg), qty!(30 deg)])
            .unwrap();
        assert!(!f.wall_contains_aabb(min, max));
    }
}
