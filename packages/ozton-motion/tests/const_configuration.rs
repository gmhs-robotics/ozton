use ozton_motion::{LtvLqr, MotionConfig, PathGuidance, Ramsete};

#[test]
fn trajectory_templates_keep_their_tracking_law() {
    const CONFIG: MotionConfig = MotionConfig::new();
    const RAMSETE: Ramsete = Ramsete::new().with_config(CONFIG);
    const LQR: LtvLqr = LtvLqr::new().with_config(CONFIG);

    assert_eq!(RAMSETE.config().path.guidance, PathGuidance::RAMSETE);
    assert_eq!(LQR.config().path.guidance, PathGuidance::LQR);
}
