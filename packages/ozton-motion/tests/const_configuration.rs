//! Const construction of physical motion templates.
use ozton_math::control::Pid;
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

#[test]
fn constant_templates_preserve_configuration() {
    assert_eq!(MOTION.tuning.linear.gains(), (4.25, 0.0, 0.50));
    assert_eq!(MOTION.limits.acceleration.value, 2.5);
    assert_eq!(MOTION.limits.angular_acceleration.value, 12.0);
}

#[test]
fn controller_templates_are_const() {
    use ozton_motion::{Basic, LtvLqr, PathGuidance, Ramsete};
    const BASIC: Basic = Basic::new().with_config(MOTION);
    const SETTINGS: MotionConfig = BASIC.config();
    assert_eq!(SETTINGS.limits.wheel_speed.value, 1.2);
    assert_eq!(Ramsete::new().with_config(MOTION).config().path.guidance, PathGuidance::RAMSETE);
    assert_eq!(LtvLqr::new().with_config(MOTION).config().path.guidance, PathGuidance::LQR);
}
