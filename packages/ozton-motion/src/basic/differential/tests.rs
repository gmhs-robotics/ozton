use ozton_math::geometry::Pose;

use super::*;

#[test]
fn match_turn_settles_with_delayed_angular_response() {
    let mut options =
        DriveOptions::differential(ozton_math::qty!(0.3292677692 m), ozton_math::qty!(0.50 m/s));
    options.heading_tolerance = ozton_math::qty!(2 deg);
    let target = DifferentialTarget::Turn {
        heading: ozton_math::qty!(1.18 rad),
    };
    let mut controller = DifferentialFeedback::new(DriveTuning::STANDARD);
    let mut heading = 0.0;
    let mut yaw_rate = 0.0;
    let mut settled = Duration::ZERO;
    let tick = Duration::from_millis(10);
    let dt = tick.as_secs_f64();

    for sample in 0..800 {
        let (wheels, ready) = controller.step(
            target,
            Feedback {
                time: tick * sample,
                pose: Pose::new(DVec2::ZERO, ozton_math::qty!((heading) rad)),
                forward_travel: Length::ZERO,
                forward_velocity: Velocity::ZERO,
                lateral_velocity: Velocity::ZERO,
                angular_velocity: ozton_math::qty!((yaw_rate) rad/s),
                ranges: [None; _],
                reset_accepted: false,
            },
            tick,
            options,
        );
        settled = if ready {
            settled + tick
        } else {
            Duration::ZERO
        };
        if settled >= options.settle {
            return;
        }

        // Model a chassis whose yaw rate follows the wheel command with a 150 ms lag.
        let commanded_rate = (wheels[1] - wheels[0]) / options.track_width.value;
        yaw_rate += (commanded_rate - yaw_rate) * (dt / 0.15);
        heading += yaw_rate * dt;
    }

    panic!("turn failed to settle within eight seconds (heading {heading}, yaw rate {yaw_rate})");
}

#[test]
fn point_brakes_inside_arrival_band_instead_of_coasting() {
    let options =
        DriveOptions::differential(ozton_math::qty!(0.3292677692 m), ozton_math::qty!(0.50 m/s));
    let mut controller = DifferentialFeedback::new(DriveTuning {
        linear: ozton_math::control::Pid::new(4.25, 0.0, 0.50, None),
        angular: ozton_math::control::Pid::new(3.0, 0.0, 0.22, None),
    });
    let (wheels, ready) = controller.step(
        DifferentialTarget::DriveToPoint {
            target: DVec2::ZERO,
            reverse: false,
            maximum_speed: options.maximum_wheel_speed,
        },
        Feedback {
            time: Duration::from_millis(10),
            pose: Pose::new(DVec2::new(0.0, -0.01), ozton_math::qty!(0 rad)),
            forward_travel: Length::ZERO,
            forward_velocity: ozton_math::qty!(0.20 m/s),
            lateral_velocity: Velocity::ZERO,
            angular_velocity: AngularVelocity::ZERO,
            ranges: [None; _],
            reset_accepted: false,
        },
        Duration::from_millis(10),
        options,
    );
    assert!(!ready);
    assert!(wheels.iter().all(|speed| *speed < 0.0), "{wheels:?}");
}

#[test]
fn point_approach_converges_with_motor_lag_and_small_overshoot() {
    let mut options =
        DriveOptions::differential(ozton_math::qty!(0.3292677692 m), ozton_math::qty!(0.50 m/s));
    options.position_tolerance = ozton_math::qty!(0.75 in);
    let tuning = DriveTuning {
        linear: ozton_math::control::Pid::new(4.25, 0.0, 0.50, None),
        angular: ozton_math::control::Pid::new(3.0, 0.0, 0.22, None),
    };
    let tick = Duration::from_millis(10);
    let dt = tick.as_secs_f64();
    for (initial, reverse) in [
        (
            Pose::new(DVec2::new(0.07375, 0.10689), ozton_math::qty!(0.08910 rad)),
            true,
        ),
        (
            Pose::new(DVec2::new(0.0, -0.24), ozton_math::qty!(0.0 rad)),
            false,
        ),
        (
            Pose::new(DVec2::new(0.0, 0.03), ozton_math::qty!(0.0 rad)),
            false,
        ),
    ] {
        let mut controller = DifferentialFeedback::new(tuning);
        let mut pose = initial;
        let mut velocity = 0.0;
        let mut yaw_rate = 0.0;
        let mut settled = Duration::ZERO;
        for sample in 0..500 {
            let (wheels, ready) = controller.step(
                DifferentialTarget::DriveToPoint {
                    target: DVec2::ZERO,
                    reverse,
                    maximum_speed: options.maximum_wheel_speed,
                },
                Feedback {
                    time: tick * sample,
                    pose,
                    forward_travel: Length::ZERO,
                    forward_velocity: ozton_math::qty!((velocity) m/s),
                    lateral_velocity: Velocity::ZERO,
                    angular_velocity: ozton_math::qty!((yaw_rate) rad/s),
                    ranges: [None; _],
                    reset_accepted: false,
                },
                tick,
                options,
            );
            settled = if ready {
                settled + tick
            } else {
                Duration::ZERO
            };
            if settled >= options.settle {
                break;
            }
            velocity += ((wheels[0] + wheels[1]) * 0.5 - velocity) * dt / 0.18;
            yaw_rate +=
                ((wheels[1] - wheels[0]) / options.track_width.value - yaw_rate) * dt / 0.15;
            pose.heading += ozton_math::qty!((yaw_rate * dt) rad);
            pose.position += ozton_math::geometry::direction(pose.heading) * velocity * dt;
        }
        assert!(
            settled >= options.settle,
            "failed from {initial:?}, at {pose:?}"
        );
    }
}

#[test]
fn slew_uses_actual_governed_wheel_targets() {
    let config = crate::MotionConfig::DEFAULT;
    let options = config.options(ozton_math::qty!(0.33 m));
    let mut controller = DifferentialFeedback::default().with_config(config);
    let dt = std::time::Duration::from_millis(10);
    for _ in 0..100 {
        controller.limit_step([0.5; 2], false, dt, options);
    }
    controller.record_applied_wheels([0.1; 2]);
    let (wheels, _) = controller.limit_step([0.5; 2], false, dt, options);
    let expected = 0.1 + config.limits.acceleration.value * dt.as_secs_f64();
    assert!(wheels.iter().all(|v| (*v - expected).abs() < 1e-10));
}

#[test]
fn breakaway_compensation_preserves_braking_and_limits() {
    let compensate = |output, error, rate, limit| {
        compensate_breakaway(output, error, rate, 0.02, 0.04, 0.1, limit, 1.0, 0.1)
    };
    assert_eq!(compensate(0.03, 0.20, 0.0, 0.5), 0.1);
    assert_eq!(compensate(-0.03, -0.20, 0.0, 0.5), -0.1);
    assert_eq!(compensate(0.03, 0.20, 0.0, 0.06), 0.06);
    assert_eq!(compensate(0.03, 0.01, 0.0, 0.5), 0.03);
    assert_eq!(compensate(-0.03, 0.20, 0.0, 0.5), -0.03);
    assert_eq!(compensate(0.03, 0.20, 0.1, 0.5), 0.03);
    assert_eq!(compensate(0.0, 0.20, 0.0, 0.5), 0.0);
    assert!(compensate(0.001, 0.021, 0.0, 0.5) < 0.02);
}

#[test]
fn minimum_speed_does_not_prevent_terminal_settling() {
    let config = crate::MotionConfig::DEFAULT.with_limits(
        crate::MotionLimits::DEFAULT
            .with_minimum_linear_speed(ozton_math::qty!(0.1 m/s))
            .with_minimum_angular_speed(ozton_math::qty!(0.5 rad/s)),
    );
    let options = config.options(ozton_math::qty!(0.33 m));
    let mut controller = DifferentialFeedback::default().with_config(config);
    let feedback = Feedback {
        time: Duration::from_millis(10),
        pose: Pose::default(),
        forward_travel: Length::ZERO,
        forward_velocity: Velocity::ZERO,
        lateral_velocity: Velocity::ZERO,
        angular_velocity: AngularVelocity::ZERO,
        ranges: [None; _],
        reset_accepted: false,
    };
    let (wheels, ready) = controller.step(
        DifferentialTarget::Turn {
            heading: Angle::default(),
        },
        feedback,
        Duration::from_millis(10),
        options,
    );
    assert!(ready);
    assert_eq!(wheels, [0.0; 2]);
}

#[test]
fn boomerang_steers_toward_carrot_and_settles_at_pose() {
    let options = DriveOptions::differential(ozton_math::qty!(0.33 m), ozton_math::qty!(0.5 m/s));
    let dt = Duration::from_millis(10);
    let target = Pose::new(DVec2::new(0.7, 0.4), ozton_math::qty!(-90 deg));
    let command = DifferentialTarget::DriveToPose {
        target,
        reverse: false,
        maximum_speed: options.maximum_wheel_speed,
    };
    let feedback = |pose| Feedback {
        time: dt,
        pose,
        forward_travel: Length::ZERO,
        forward_velocity: Velocity::ZERO,
        lateral_velocity: Velocity::ZERO,
        angular_velocity: AngularVelocity::ZERO,
        ranges: [None; _],
        reset_accepted: false,
    };
    let mut controller = DifferentialFeedback::default();
    let (wheels, ready) = controller.step(command, feedback(Pose::default()), dt, options);
    assert!(!ready);
    assert!(wheels[0] > wheels[1], "{wheels:?}");
    assert!(wheels[0] + wheels[1] > 0.0);

    let mut controller = DifferentialFeedback::default();
    let (wheels, ready) = controller.step(command, feedback(target), dt, options);
    assert!(ready);
    assert_eq!(wheels, [0.0; 2]);
}
