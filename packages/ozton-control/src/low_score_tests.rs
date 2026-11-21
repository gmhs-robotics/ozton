use ozton_math::uom::si::angle::degree;

use super::*;

const EJECT_TIME: Duration = Duration::from_millis(1500);
const TRAVEL_TIMEOUT: Duration = Duration::from_secs(8);

fn angle(value: f64) -> Angle {
    Angle::new::<degree>(value)
}

fn limits() -> LowScoreLimits {
    LowScoreLimits {
        lift: PositionLimits::new(Some((angle(0.0), angle(700.0))), angle(3.0)),
        pivot: PositionLimits::new(Some((angle(0.0), angle(368.0))), angle(3.0)),
        low_height: angle(0.0),
        eject_time: EJECT_TIME,
        travel_timeout: TRAVEL_TIMEOUT,
    }
}

const TICK: Duration = Duration::from_millis(10);

#[test]
fn lowers_ejects_then_restores_height_without_repeating_held_button() {
    let mut score = LowScore::default();
    let first = score
        .update(true, false, angle(500.0), angle(360.0), limits(), TICK)
        .unwrap()
        .unwrap();
    assert_eq!(first.lift_target, angle(0.0));
    assert_eq!(first.pivot_target, angle(360.0));
    assert_eq!(first.intake_direction, 0);
    let eject = score
        .update(true, false, angle(0.0), angle(360.0), limits(), TICK)
        .unwrap()
        .unwrap();
    assert_eq!(eject.intake_direction, -1);
    assert_eq!(eject.pivot_target, angle(360.0));
    let still_ejecting = score
        .update(
            true,
            false,
            angle(0.0),
            angle(350.0),
            limits(),
            Duration::from_millis(1490),
        )
        .unwrap()
        .unwrap();
    assert_eq!(still_ejecting.intake_direction, -1);
    assert_eq!(still_ejecting.lift_target, angle(0.0));
    let returning = score
        .update(true, false, angle(0.0), angle(350.0), limits(), TICK)
        .unwrap()
        .unwrap();
    assert_eq!(returning.lift_target, angle(500.0));
    assert_eq!(returning.intake_direction, 0);
    score
        .update(true, false, angle(500.0), angle(360.0), limits(), TICK)
        .unwrap();
    assert!(
        score
            .update(true, false, angle(500.0), angle(360.0), limits(), TICK)
            .unwrap()
            .is_none()
    );
}

#[test]
fn held_b_starts_after_manual_release_without_restarting_cancelled_cycle() {
    let mut score = LowScore::default();
    let c = limits();
    for height in [400.0, 450.0, 500.0] {
        assert!(
            score
                .update(true, true, angle(height), angle(360.0), c, TICK)
                .unwrap()
                .is_none()
        );
    }
    for height in [500.0, 400.0] {
        let lower = score
            .update(true, false, angle(height), angle(360.0), c, TICK)
            .unwrap()
            .unwrap();
        assert_eq!(lower.lift_target, angle(0.0));
    }
    assert!(
        score
            .update(true, true, angle(400.0), angle(360.0), c, TICK)
            .unwrap()
            .is_none()
    );
    assert!(
        score
            .update(true, false, angle(400.0), angle(360.0), c, TICK)
            .unwrap()
            .is_none()
    );
    score
        .update(false, false, angle(400.0), angle(360.0), c, TICK)
        .unwrap();
    assert!(
        score
            .update(true, false, angle(400.0), angle(360.0), c, TICK)
            .unwrap()
            .is_some()
    );
}

#[test]
fn b_keeps_the_captured_pivot_target_throughout() {
    let c = limits();
    for pivot in [300.0, 360.0] {
        let mut score = LowScore::default();
        let first = score
            .update(true, false, angle(500.0), angle(pivot), c, TICK)
            .unwrap()
            .unwrap();
        assert_eq!(first.lift_target, angle(0.0));
        assert_eq!(first.pivot_target, angle(pivot));
        let eject = score
            .update(false, false, angle(0.0), first.pivot_target, c, TICK)
            .unwrap()
            .unwrap();
        assert_eq!(eject.intake_direction, -1);
        let returning = score
            .update(false, false, angle(0.0), first.pivot_target, c, EJECT_TIME)
            .unwrap()
            .unwrap();
        assert_eq!(returning.lift_target, angle(500.0));
        assert_eq!(returning.pivot_target, angle(pivot));
        assert_eq!(returning.intake_direction, 0);
    }
}

#[test]
fn second_press_manual_input_and_timeout_cancel() {
    let mut score = LowScore::default();
    score
        .update(true, false, angle(500.0), angle(360.0), limits(), TICK)
        .unwrap();
    score
        .update(false, false, angle(400.0), angle(360.0), limits(), TICK)
        .unwrap();
    assert!(
        score
            .update(true, false, angle(400.0), angle(360.0), limits(), TICK)
            .unwrap()
            .is_none()
    );
    score
        .update(false, false, angle(400.0), angle(360.0), limits(), TICK)
        .unwrap();
    score
        .update(true, false, angle(400.0), angle(360.0), limits(), TICK)
        .unwrap();
    assert!(
        score
            .update(false, true, angle(400.0), angle(360.0), limits(), TICK)
            .unwrap()
            .is_none()
    );
    score
        .update(true, false, angle(400.0), angle(360.0), limits(), TICK)
        .unwrap();
    assert!(
        score
            .update(
                false,
                false,
                angle(400.0),
                angle(360.0),
                limits(),
                TRAVEL_TIMEOUT
            )
            .is_err()
    );
}
#[test]
fn return_waits_for_pivot_and_times_out_if_it_cannot_restore_pose() {
    let mut score = LowScore::default();
    let c = limits();
    score
        .update(true, false, angle(500.0), angle(300.0), c, TICK)
        .unwrap();
    score
        .update(false, false, angle(0.0), angle(320.0), c, TICK)
        .unwrap();
    score
        .update(false, false, angle(0.0), angle(320.0), c, EJECT_TIME)
        .unwrap();
    let returning = score
        .update(false, false, angle(500.0), angle(320.0), c, TICK)
        .unwrap()
        .unwrap();
    assert_eq!(returning.lift_target, angle(500.0));
    assert_eq!(returning.pivot_target, angle(300.0));
    assert_eq!(returning.intake_direction, 0);
    assert!(score.cycle.is_some());
    assert!(
        score
            .update(false, false, angle(500.0), angle(320.0), c, TRAVEL_TIMEOUT)
            .is_err()
    );
    assert!(score.cycle.is_none());
}

#[test]
fn b_waits_for_bottom_before_ejecting_even_with_pivot_drift() {
    let mut score = LowScore::default();
    let c = limits();
    score
        .update(true, false, angle(500.0), angle(360.0), c, TICK)
        .unwrap();
    let lowering = score
        .update(false, false, angle(5.0), angle(360.0), c, TICK)
        .unwrap()
        .unwrap();
    assert_eq!(lowering.intake_direction, 0);
    assert!(!lowering.hold_lift);
    // Crossing zero starts ejection even if the pivot drifted.
    let at_bottom = score
        .update(false, false, angle(-1.0), angle(340.0), c, TICK)
        .unwrap()
        .unwrap();
    assert_eq!(at_bottom.intake_direction, -1);
    assert!(at_bottom.hold_lift);
    // Encoder rebound must not undo bottom arrival.
    let ejecting = score
        .update(false, false, angle(2.0), angle(360.0), c, TICK)
        .unwrap()
        .unwrap();
    assert_eq!(ejecting.intake_direction, -1);
    assert!(ejecting.hold_lift);
}
