use ozton_math::{
    angle,
    geometry::Pose,
    length,
    units::{angular_velocity, velocity},
};

use super::*;

#[test]
fn match_turn_settles_with_delayed_angular_response() {
    let mut options =
        DriveOptions::differential(length!(0.3292677692 m), velocity!(0.50, meter_per_second));
    options.heading_tolerance = angle!(2 deg);
    let target = DifferentialTarget::Turn {
        heading: angle!(1.18 rad),
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
                pose: Pose::new(DVec2::ZERO, angle!(heading, rad)),
                forward_travel: Length::ZERO,
                forward_velocity: Velocity::ZERO,
                lateral_velocity: Velocity::ZERO,
                angular_velocity: angular_velocity!(yaw_rate, radian_per_second),
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
        DriveOptions::differential(length!(0.3292677692 m), velocity!(0.50, meter_per_second));
    let tick = Duration::from_millis(10);
    let (wheels, ready) = DifferentialFeedback::default().step(
        DifferentialTarget::DriveToPoint {
            target: DVec2::ZERO,
            reverse: true,
            maximum_speed: options.maximum_wheel_speed,
        },
        Feedback {
            time: tick,
            pose: Pose::new(DVec2::new(0.07375, 0.10689), angle!(0.08910 rad)),
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
