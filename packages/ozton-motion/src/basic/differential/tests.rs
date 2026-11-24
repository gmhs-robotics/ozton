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
fn point_approach_turns_before_driving_at_the_logged_heading_error() {
    let options =
        DriveOptions::differential(ozton_math::qty!(0.3292677692 m), ozton_math::qty!(0.50 m/s));
    let tick = Duration::from_millis(10);
    let (wheels, ready) = DifferentialFeedback::default().step(
        DifferentialTarget::DriveToPoint {
            target: DVec2::ZERO,
            reverse: true,
            maximum_speed: options.maximum_wheel_speed,
        },
        Feedback {
            time: tick,
            pose: Pose::new(DVec2::new(0.07375, 0.10689), ozton_math::qty!(0.08910 rad)),
            forward_travel: Length::ZERO,
            forward_velocity: Velocity::ZERO,
            lateral_velocity: Velocity::ZERO,
            angular_velocity: AngularVelocity::ZERO,
            ranges: [None; _],
            reset_accepted: false,
        },
        tick,
        options,
    );
    assert!(!ready);
    assert!(wheels[0].abs() > 0.01);
    assert!((wheels[0] + wheels[1]).abs() < 1e-12, "{wheels:?}");
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
        (Pose::new(DVec2::new(0.0, -0.24), ozton_math::qty!(0.0 rad)), false),
        (Pose::new(DVec2::new(0.0, 0.03), ozton_math::qty!(0.0 rad)), false),
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
