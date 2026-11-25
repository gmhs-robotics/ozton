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
    assert_eq!(
        Ramsete::new().with_config(MOTION).config().path.guidance,
        PathGuidance::RAMSETE
    );
    assert_eq!(
        LtvLqr::new().with_config(MOTION).config().path.guidance,
        PathGuidance::LQR
    );
}

#[test]
fn controller_capabilities_are_independent() {
    use ozton_motion::{Basic, Boomerang, LtvLqr, PurePursuit, Ramsete, capabilities::*};
    fn distance<T: DriveDistance>() {}
    fn pose<T: MoveToPose>() {}
    fn trajectory<T: FollowPath>() {}
    distance::<Basic>();
    pose::<Basic>();
    pose::<Boomerang>();
    trajectory::<PurePursuit>();
    trajectory::<Ramsete>();
    trajectory::<LtvLqr>();
}

#[test]
fn pursuit_enhancements_are_explicit_and_independent() {
    use core::time::Duration;

    use ozton_math::qty;
    use ozton_motion::{Boomerang, PurePursuit, TerminalGuidance};

    const PLAIN: PurePursuit = PurePursuit::new().with_lookahead(qty!(0.23 m));
    const ADAPTIVE: PurePursuit = PLAIN.with_adaptive_lookahead(Duration::from_millis(320));
    const PREDICTIVE: PurePursuit = PLAIN
        .with_prediction_latency(Duration::from_millis(125), Duration::from_millis(160))
        .with_heading_recovery(true)
        .with_terminal(TerminalGuidance::POLAR);

    assert_eq!(PLAIN.config().path.lookahead.value, 0.23);
    assert_eq!(PLAIN.config().path.adaptive_lookahead, None);
    assert_eq!(PLAIN.config().path.prediction_latency, None);
    assert!(!PLAIN.config().path.heading_recovery);
    assert_eq!(ADAPTIVE.config().path.prediction_latency, None);
    assert_eq!(
        ADAPTIVE.config().path.adaptive_lookahead,
        Some(Duration::from_millis(320))
    );
    assert_eq!(PREDICTIVE.config().path.adaptive_lookahead, None);
    assert!(PREDICTIVE.config().path.heading_recovery);
    assert_eq!(PREDICTIVE.config().path.terminal, TerminalGuidance::POLAR);
    assert_eq!(
        PREDICTIVE
            .without_prediction()
            .config()
            .path
            .prediction_latency,
        None
    );
    assert_eq!(
        ADAPTIVE
            .without_adaptive_lookahead()
            .config()
            .path
            .adaptive_lookahead,
        None
    );
    assert_eq!(
        Boomerang::new()
            .with_lead(0.4)
            .config()
            .approach
            .boomerang_lead,
        0.4
    );
}
