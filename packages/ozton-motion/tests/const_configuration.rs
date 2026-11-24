//! Const construction of physical motion templates.
use ozton_math::{control::Pid, following::Ramsete};
use ozton_motion::{DriveTuning, MotionConfig, MotionLimits};

const PID: Pid = Pid::new(4.25, 0.0, 0.50, None);
const MOTION: MotionConfig = MotionConfig::new()
    .with_tuning(DriveTuning::new().with_linear_controller(PID))
    .with_limits(
        MotionLimits::new()
            .with_acceleration(ozton_math::qty!(2.5 m/s^2))
            .with_angular_acceleration(ozton_math::qty!(12 rad/s^2))
            .with_wheel_speed(ozton_math::qty!(1.2 m/s)),
    );
const RAMSETE: Ramsete = Ramsete::new().with_b(2.5).with_zeta(0.8);

#[test]
fn constant_templates_preserve_configuration() {
    assert_eq!(MOTION.tuning.linear.gains(), (4.25, 0.0, 0.50));
    assert_eq!(MOTION.limits.acceleration.value, 2.5);
    assert_eq!(MOTION.limits.angular_acceleration.value, 12.0);
    assert_eq!(RAMSETE.zeta, 0.8);
}

#[cfg(feature = "vex")]
#[test]
fn seeking_template_is_const() {
    use ozton_motion::Seeking;
    const SEEKING: Seeking = Seeking::new().with_config(MOTION);
    const SETTINGS: MotionConfig = SEEKING.config();
    assert_eq!(SETTINGS.limits.wheel_speed.value, 1.2);
}
