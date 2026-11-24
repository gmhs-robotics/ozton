//! These declarations must compile without any runtime construction.
use std::time::Duration;

use ozton_control::loops::{AngularPid, Pid};
use ozton_math::{
    angle, length,
    units::{acceleration, angular_velocity, velocity},
};
use ozton_motion::{
    Basic, MotionConfig, PurePursuit, pursuit::PursuitTerminalConfig, ramsete::Ramsete,
};

const PID: Pid = Pid::new(1.0, 0.0, 0.0, None)
    .with_kd(0.06)
    .with_output_limit(0.65);
const BASIC: Basic = Basic::new()
    .with_linear_controller(PID)
    .with_angular_controller(AngularPid::new(1.6, 0.0, 0.08, None))
    .with_linear_error_tolerance(length!(0.75 in))
    .with_angular_error_tolerance(angle!(2 deg))
    .with_linear_velocity_tolerance(velocity!(0.04, meter_per_second))
    .with_angular_velocity_tolerance(angular_velocity!(0.12, radian_per_second))
    .with_timeout(Duration::from_secs(6));
const PURSUIT: PurePursuit = PurePursuit::new()
    .with_track_width(length!(13 in))
    .with_lookahead_distance(length!(6 in))
    .with_terminal_config(PursuitTerminalConfig::new().with_linear_controller(PID));
const MOTION: MotionConfig = MotionConfig::new()
    .with_acceleration(acceleration!(2.5, meter_per_second_squared))
    .with_wheel_speed(velocity!(1.2, meter_per_second));
const RAMSETE: Ramsete = Ramsete::new().with_b(2.5).with_zeta(0.8);

#[test]
fn constant_templates_preserve_configuration() {
    assert_eq!(BASIC.linear_controller.gains(), (1.0, 0.0, 0.06));
    assert_eq!(BASIC.linear_controller.output_limit(), Some(0.65));
    assert_eq!(BASIC.timeout, Some(Duration::from_secs(6)));
    assert_eq!(PURSUIT.track_width, length!(13 in));
    assert_eq!(PURSUIT.terminal_config.linear, PID);
    assert_eq!(MOTION.acceleration.value, 2.5);
    assert_eq!(RAMSETE.zeta, 0.8);
}

#[cfg(feature = "vex")]
#[test]
fn seeking_templates_are_const() {
    use ozton_motion::{PoseSeeking, Seeking};
    const SEEKING: Seeking = Seeking::new()
        .with_linear_controller(PID)
        .with_lateral_gains(1.8, 0.0, 0.06)
        .with_error_tolerance(length!(1 in));
    const POSE: PoseSeeking = PoseSeeking::new()
        .with_linear_gains(4.25, 0.0, 0.5)
        .with_angular_gains(3.0, 0.0, 0.22)
        .with_acceleration(acceleration!(2.5, meter_per_second_squared))
        .with_wheel_speed(velocity!(1.2, meter_per_second));
    const SETTINGS: MotionConfig = POSE.config();
    assert_eq!(SEEKING.linear_controller, PID);
    assert_eq!(SETTINGS.tuning.linear.gains(), (4.25, 0.0, 0.5));
    assert_eq!(SETTINGS.wheel_speed.value, 1.2);
}
