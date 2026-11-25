//! Predictive stopping envelope and curvature-preserving command governor.
use std::cell::Cell;

use ozton_diagnostics::{log_event, log_row};
use ozton_localization::Feedback;
use ozton_math::{
    geometry::DVec2,
    path::{
        braking::{RateBounds, position_padding},
        clearance::{RectangularClearance, WallContact},
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
    limit_wall_command_at_contact(
        field,
        feedback,
        wheels,
        options,
        config,
        uncertainty,
        covariance,
        previous,
        None,
    )
}

/// The explicit contact policy is used only for departure from a surveyed
/// physical wall. Virtual exclusions retain the ordinary hard envelope.
pub(super) fn limit_wall_command_at_contact(
    field: Option<RectangularClearance>,
    feedback: Feedback,
    wheels: [f64; 2],
    options: DriveOptions,
    config: MotionConfig,
    uncertainty: [f64; 3],
    covariance: Option<glam::DMat2>,
    previous: [f64; 2],
    contact: Option<WallContact>,
) -> [f64; 2] {
    let Some(field) = field else {
        return wheels;
    };
    let [v, w] = body_rates(wheels, options.track_width.value);
    let previous = body_rates(previous, options.track_width.value);
    let delay = reaction(config) + config.safety.wall_preview.as_secs_f64();
    let projected = |v, w| {
        bounds(
            field,
            feedback,
            v,
            w,
            previous,
            uncertainty,
            covariance,
            delay,
            config,
        )
    };
    let full = projected(v, w);
    if full.is_some_and(|(min, max)| field.contains_aabb(min, max)) {
        return wheels;
    }
    let baseline = projected(0.0, 0.0);
    let clear = |v, w| {
        projected(v, w).is_some_and(|(min, max)| {
            field.contains_aabb(min, max)
                || baseline.is_some_and(|(base_min, base_max)| {
                    let permitted = field.wall_recovery_contains(min, max, base_min, base_max)
                        || contact.is_some_and(|wall| {
                            field.contact_recovery_contains(wall, min, max, base_min, base_max)
                                && field.contact_inward(wall, feedback.pose, [0.0, v, w], [0.0; 3])
                        });
                    permitted
                        && field.obstacle_free_aabb(min, max)
                        && field.wall_recovery_direction(feedback.pose, v, w, base_min, base_max)
                })
        })
    };
    let scale = |v, w| {
        if clear(v, w) {
            return 1.0;
        }
        let mut low = 0.0;
        let mut high = 1.0;
        for _ in 0..12 {
            let middle = (low + high) * 0.5;
            if clear(v * middle, w * middle) {
                low = middle;
            } else {
                high = middle;
            }
        }
        low
    };
    let low = scale(v, w);
    // At a surveyed flush contact a yaw correction can be infeasible even
    // though straight translation is safe. Project onto the straight inward
    // command if it makes more progress; the PID sees the actual applied rate.
    // This is restricted to the explicitly declared straight departure stage.
    let straight = if contact.is_some() && v != 0.0 && w != 0.0 && wheels[0] * wheels[1] >= 0.0 {
        scale(v, 0.0)
    } else {
        0.0
    };
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
        "reason={reason},sample_us={},scale={low:.5},straight_scale={straight:.5},requested_left_mps={:.5},requested_right_mps={:.5},prior_v_mps={:.5},prior_yaw_radps={:.5},sigma_lateral={:.5},sigma_forward={:.5},sigma_yaw={:.5},min_x_m={:.5},min_y_m={:.5},max_x_m={:.5},max_y_m={:.5}",
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
    if straight > low {
        [v * straight; 2]
    } else {
        wheels.map(|speed| speed * low)
    }
}

/// Release the one-shot contact policy as soon as the ordinary predicted
/// stopping envelope certifies the preferred clearance.
pub(super) fn contact_released(
    field: RectangularClearance,
    feedback: Feedback,
    config: MotionConfig,
    uncertainty: [f64; 3],
    covariance: Option<glam::DMat2>,
    previous: [f64; 2],
) -> bool {
    bounds(
        field,
        feedback,
        0.0,
        0.0,
        previous,
        uncertainty,
        covariance,
        reaction(config) + config.safety.wall_preview.as_secs_f64(),
        config,
    )
    .is_some_and(|(min, max)| field.contains_aabb(min, max))
}

pub(super) fn safe_feedback(
    field: RectangularClearance,
    feedback: Feedback,
    config: MotionConfig,
    uncertainty: [f64; 3],
    covariance: Option<glam::DMat2>,
    previous: [f64; 2],
) -> bool {
    safe_feedback_at_contact(
        field,
        feedback,
        config,
        uncertainty,
        covariance,
        previous,
        None,
    )
}

pub(super) fn safe_feedback_at_contact(
    field: RectangularClearance,
    feedback: Feedback,
    config: MotionConfig,
    uncertainty: [f64; 3],
    covariance: Option<glam::DMat2>,
    previous: [f64; 2],
    contact: Option<WallContact>,
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
    let wall_safe = projected.is_some_and(|(min, max)| {
        field.physical_wall_contains_aabb(min, max)
            || contact.is_some_and(|wall| {
                field.contact_recovery_contains(wall, min, max, min, max)
                    && field.contact_inward(
                        wall,
                        feedback.pose,
                        [feedback.lateral_velocity.value, v, w],
                        uncertainty.map(|sigma| sigma * config.safety.braking_sigma),
                    )
                    && field.contact_inward(
                        wall,
                        feedback.pose,
                        [0.0, previous[0], previous[1]],
                        [0.0; 3],
                    )
            })
    });
    let reason = if !field.contains_physical_pose(feedback.pose) {
        "current_footprint"
    } else if !wall_safe {
        "wall_stopping_envelope"
    } else if !projected.is_some_and(|(min, max)| field.obstacle_free_aabb(min, max)) {
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
    fn reserve_recovery_allows_inward_motion_but_preserves_hard_boundaries() {
        let footprint = Footprint::new(qty!(0.2 m), qty!(0.4 m)).unwrap();
        let field = RectangularClearance::new(footprint, DVec2::ONE, qty!(0.1 m)).unwrap();
        let config = MotionConfig::DEFAULT;
        let options = DriveOptions::differential(qty!(0.3 m), qty!(0.5 m/s));
        let mut feedback = sample(0.0);
        feedback.pose = Pose::new(DVec2::new(0.0, -0.76), qty!(0 rad));
        feedback.angular_velocity = qty!(0 rad/s);
        let uncertainty = [0.01; 3];
        let covariance = Some(glam::DMat2::from_diagonal(DVec2::splat(0.000025)));
        let inward = [qty!(0.2 m/s); 2];
        let governor = StoppingGovernor::new(Some(field), options, config);
        assert_eq!(
            governor.govern(feedback, uncertainty, covariance, inward),
            Some(inward)
        );
        let governor = StoppingGovernor::new(Some(field), options, config);
        assert_eq!(
            governor.govern(feedback, uncertainty, covariance, [qty!(-0.2 m/s); 2]),
            Some([qty!(0 m/s); 2])
        );
        let field = field
            .with_obstacle(DVec2::new(-0.2, -0.5), DVec2::new(0.2, -0.4))
            .unwrap();
        let governor = StoppingGovernor::new(Some(field), options, config);
        assert!(
            governor
                .govern(feedback, uncertainty, covariance, inward)
                .is_none()
        );
    }

    #[test]
    fn surveyed_contact_allows_inward_motion_but_never_an_outward_command() {
        let field = RectangularClearance::new(
            Footprint::new(qty!(0.2 m), qty!(0.4 m)).unwrap(),
            DVec2::ONE,
            qty!(1 in),
        )
        .unwrap();
        let mut feedback = sample(0.0);
        feedback.pose = Pose::new(DVec2::new(0.0, -0.8), qty!(0 rad));
        feedback.angular_velocity = qty!(0 rad/s);
        let config = MotionConfig::DEFAULT;
        let options = DriveOptions::differential(qty!(0.3 m), qty!(0.5 m/s));
        let uncertainty = [0.11, 0.061, 0.0474];
        let covariance = Some(glam::DMat2::from_diagonal(DVec2::new(0.00065, 0.0)));
        let contact = Some(WallContact::Rear);
        assert!(!safe_feedback(
            field,
            feedback,
            config,
            uncertainty,
            covariance,
            [0.0; 2]
        ));
        assert!(safe_feedback_at_contact(
            field,
            feedback,
            config,
            uncertainty,
            covariance,
            [0.0; 2],
            contact
        ));
        let apply = |wheels| {
            limit_wall_command_at_contact(
                Some(field),
                feedback,
                wheels,
                options,
                config,
                uncertainty,
                covariance,
                [0.0; 2],
                contact,
            )
        };
        assert_eq!(apply([0.2; 2]), [0.2; 2]);
        assert_eq!(apply([-0.2; 2]), [0.0; 2]);
        assert_eq!(apply([-0.2, 0.2]), [0.0; 2]);
        let corrected = apply([0.18, 0.20]);
        assert!(corrected[0] > 0.0 && corrected[1] > 0.0);
        assert!(field.contact_inward(
            WallContact::Rear,
            feedback.pose,
            [
                0.0,
                (corrected[0] + corrected[1]) * 0.5,
                (corrected[1] - corrected[0]) / options.track_width.value
            ],
            [0.0; 3]
        ));
        let blocked = field
            .with_obstacle(DVec2::new(-0.2, -0.55), DVec2::new(0.2, -0.5))
            .unwrap();
        assert!(!safe_feedback_at_contact(
            blocked,
            feedback,
            config,
            uncertainty,
            covariance,
            [0.0; 2],
            contact
        ));
        feedback.forward_velocity = qty!(-0.4 m/s);
        assert!(!safe_feedback_at_contact(
            field,
            feedback,
            config,
            uncertainty,
            covariance,
            [0.0; 2],
            contact
        ));
    }

    #[test]
    fn override_rear_contact_completes_the_standard_distance_controller_in_simulation() {
        use ozton_math::control::Pid;

        use crate::basic::differential::{DifferentialFeedback, DifferentialTarget, DriveTuning};
        let footprint = Footprint::from_extents(
            qty!(7.55 in),
            qty!(7.55 in),
            qty!(7.761804 in),
            qty!(7.238196 in),
        )
        .unwrap();
        let bounds = DVec2::new(qty!(45.5 in).value, qty!(48.5 in).value);
        let (min, _) = footprint.bounds(Pose::default());
        let start = Pose::new(
            -bounds + DVec2::new(qty!(12 in).value, 0.0) - min,
            qty!(0 rad),
        );
        let field = RectangularClearance::new(footprint, bounds, qty!(1 in)).unwrap();
        assert!(field.touches_wall(start, WallContact::Rear));
        let config = MotionConfig::DEFAULT
            .with_tuning(
                DriveTuning::new()
                    .with_linear_controller(Pid::new(4.25, 0.0, 0.50, None))
                    .with_angular_controller(Pid::new(3.0, 0.0, 0.22, None)),
            )
            .with_limits(
                crate::MotionLimits::DEFAULT
                    .with_wheel_speed(qty!(0.20 m/s))
                    .with_deceleration(qty!(0.55 m/s^2)),
            )
            .with_safety(
                crate::ClearanceConfig::DEFAULT
                    .with_clearance(qty!(1 in))
                    .with_forward_braking(qty!(0.55 m/s^2))
                    .with_yaw_braking(qty!(3 rad/s^2)),
            );
        let options = config.options(qty!(0.3292677692 m));
        let target = DifferentialTarget::DriveDistance {
            meters: qty!(12 in),
            heading: Some(start.heading),
            maximum_speed: options.maximum_wheel_speed,
        };
        let mut controller = DifferentialFeedback::new(config.tuning).with_config(config);
        let mut feedback = sample(0.0);
        feedback.pose = start;
        feedback.angular_velocity = qty!(0 rad/s);
        let uncertainty = [0.111, 0.061, 0.0474];
        let mut contact = Some(WallContact::Rear);
        let dt = Duration::from_millis(10);
        let mut settled = Duration::ZERO;
        for _ in 0..1000 {
            let travel = feedback.forward_travel.value.abs();
            let diffusion = 0.04_f64.powi(2) * travel;
            let covariance = Some(glam::DMat2::from_diagonal(DVec2::new(
                qty!(1 in).value.powi(2) + diffusion,
                diffusion,
            )));
            let previous = controller.applied_wheels();
            let rates = body_rates(previous, options.track_width.value);
            if contact.is_some()
                && contact_released(field, feedback, config, uncertainty, covariance, rates)
            {
                contact = None;
            }
            assert!(
                safe_feedback_at_contact(
                    field,
                    feedback,
                    config,
                    uncertainty,
                    covariance,
                    rates,
                    contact
                ),
                "feedback={feedback:?}, contact={contact:?}"
            );
            let (requested, ready) = controller.step(target, feedback, dt, options);
            let applied = limit_wall_command_at_contact(
                Some(field),
                feedback,
                requested,
                options,
                config,
                uncertainty,
                covariance,
                previous,
                contact,
            );
            controller.record_applied_wheels(applied);
            settled = if ready { settled + dt } else { Duration::ZERO };
            if settled >= options.settle {
                assert!(contact.is_none());
                assert!(
                    (feedback.forward_travel.value - qty!(12 in).value).abs()
                        <= options.position_tolerance.value
                );
                return;
            }
            // A first-order motor-lag simulation, not a physical motor qualification.
            let [linear, yaw] = body_rates(applied, options.track_width.value);
            feedback.forward_velocity.value +=
                (linear - feedback.forward_velocity.value) * dt.as_secs_f64() / 0.15;
            feedback.angular_velocity.value +=
                (yaw - feedback.angular_velocity.value) * dt.as_secs_f64() / 0.15;
            let advance = feedback.forward_velocity.value * dt.as_secs_f64();
            feedback.forward_travel.value += advance;
            feedback.pose.position +=
                ozton_math::geometry::direction(feedback.pose.heading) * advance;
            feedback.pose.heading.value += feedback.angular_velocity.value * dt.as_secs_f64();
            feedback.time += dt;
            assert!(field.contains_physical_pose(feedback.pose));
        }
        panic!("wall departure did not settle: {feedback:?}");
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
