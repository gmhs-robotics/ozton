use super::*;

#[test]
fn dsr_combines_independent_packets_without_faking_velocity() {
    let mut localizer = localizer(Estimator::Odometry, false);
    assert_eq!(
        localizer.update(snapshot(10), &[]).unwrap_err(),
        Fault::Initializing
    );
    localizer.request_distance_reset();
    let waiting = localizer
        .update(snapshot(20), &[reading(19, false), None])
        .unwrap();
    assert!(!waiting.distance_reset);
    // The rest dwell precedes packet collection. Old packets cannot satisfy it.
    for time in [60, 100, 140] {
        assert!(
            !localizer
                .update(snapshot(time), &[reading(time, false), None])
                .unwrap()
                .distance_reset
        );
    }
    let fixed = localizer
        .update(snapshot(190), &[reading(180, false), reading(189, false)])
        .unwrap();
    assert!(fixed.distance_reset);
    assert!(fixed.pose.position.length() < 1e-10);
    assert_eq!(fixed.forward_velocity, 0.0);
    assert_eq!(localizer.forward_travel(), 0.0);
    let duplicate = localizer
        .update(snapshot(200), &[reading(180, false), reading(189, false)])
        .unwrap();
    assert_eq!(
        duplicate.ranges[0].unwrap().constraint,
        Err(Rejection::Duplicate)
    );
}

#[test]
fn full_dsr_waits_for_second_axis_and_retains_requirement_after_motion() {
    let mut tracking = localizer(Estimator::Odometry, false);
    tracking.update(snapshot(10), &[]).unwrap_err();
    assert!(!tracking.request_distance_reset_axes(0));
    assert!(!tracking.request_distance_reset_axes(3));
    assert!(tracking.request_distance_reset_axes(2));
    for time in (20..=240).step_by(10) {
        // Repeated requests and the runner's ordinary reset request are idempotent.
        assert!(tracking.request_distance_reset_axes(2));
        tracking.request_distance_reset();
        let update = tracking
            .update(snapshot(time), &[reading(time, false), None])
            .unwrap();
        assert!(!update.distance_reset);
    }
    let mut moved = snapshot(250);
    moved.raw.drive = [Some(Length::new::<meter>(0.01)); 2];
    assert!(
        !tracking
            .update(moved, &[reading(250, false), None])
            .unwrap()
            .distance_reset
    );
    let before = tracking.pose();
    // The velocity estimator must also settle before the stationary dwell
    // and collection windows begin. A zero encoder delta alone is not proof of rest.
    for time in (260..=600).step_by(10) {
        let mut stopped = snapshot(time);
        stopped.raw.drive = moved.raw.drive;
        assert!(
            !tracking
                .update(stopped, &[reading(time, false), None])
                .unwrap()
                .distance_reset
        );
    }
    assert_eq!(
        tracking.pose(),
        before,
        "a partial reset must not be applied"
    );
    let mut stopped = snapshot(610);
    stopped.raw.drive = moved.raw.drive;
    let fixed = tracking.update(stopped, &[reading(610, false); 2]).unwrap();
    assert!(fixed.distance_reset);
    assert_eq!(fixed.distance_reset_axes, 2);
    assert!(fixed.pose.position.length() < 1e-10);
}

#[test]
fn coherent_wall_reset_can_correct_a_large_pose_error() {
    let mut localization = localizer(Estimator::Odometry, false);
    localization.update(snapshot(10), &[]).unwrap_err();
    assert!(localization.request_distance_reset_axes(2));
    let before = localization.pose();
    for time in (20..=210).step_by(10) {
        let farther = Some(RangeObservation {
            distance: Length::new::<meter>(1.45),
            ..reading(time, false).unwrap()
        });
        let update = localization.update(snapshot(time), &[farther; 2]).unwrap();
        if update.distance_reset {
            assert_eq!(update.distance_reset_axes, 2);
            assert!(update.pose.position.distance(before.position) > 0.5);
            return;
        }
    }
    panic!("coherent range readings should correct the pose");
}

#[test]
fn dsr_rejects_translation_lateral_and_rotation_until_rest_dwell_and_fresh_collection() {
    for direction in 0..3 {
        let mut tracker = localizer(Estimator::Odometry, false);
        tracker.update(snapshot(0), &[]).unwrap_err();
        tracker.request_distance_reset();
        let mut raw = snapshot(0);
        for time in (10..=200).step_by(10) {
            raw.raw.time = Duration::from_millis(time);
            match direction {
                0 => {
                    raw.raw.drive = [Some(Length::new::<meter>(time as f64 / 1000.0)); 2];
                }
                1 => {
                    raw.lateral = Some(Length::new::<meter>(time as f64 / 1000.0));
                }
                _ => {
                    raw.raw.imu = Some(Angle::new::<radian>(time as f64 / 1000.0));
                }
            }
            let result = tracker
                .update(raw, &[reading(time, false), reading(time, false)])
                .unwrap();
            assert!(
                !result.distance_reset,
                "moving DSR accepted on axis {direction}"
            );
        }
    }
}

#[test]
fn redundant_conflicting_beams_reject_independent_of_sensor_order() {
    let opposite = RangeSensorConfig {
        mount: Pose::new(
            ozton_math::geometry::point(Length::new::<meter>(0.0), Length::new::<meter>(0.0)),
            Angle::new::<radian>(std::f64::consts::PI),
        ),
        surface: None,
        ..SENSOR
    };
    let forward = RangeSensorConfig {
        surface: None,
        ..SENSOR
    };
    let map = [
        Surface::new(
            ozton_math::geometry::point(Length::new::<meter>(1.0), Length::new::<meter>(1.0)),
            ozton_math::geometry::point(Length::new::<meter>(-1.0), Length::new::<meter>(1.0)),
            ozton_math::qty!(1 m),
        ),
        Surface::new(
            ozton_math::geometry::point(Length::new::<meter>(-1.0), Length::new::<meter>(-1.0)),
            ozton_math::geometry::point(Length::new::<meter>(1.0), Length::new::<meter>(-1.0)),
            ozton_math::qty!(1 m),
        ),
    ];
    for sensors in [[forward, opposite], [opposite, forward]] {
        let mut tracker = Fixture::new(
            Config {
                maximum_gap: MAXIMUM_GAP,
                estimator: Estimator::Odometry,
                shadow: false,
            },
            Pose::default(),
            &map,
            &sensors,
        )
        .unwrap();
        tracker.update(snapshot(0), &[]).unwrap_err();
        tracker.request_distance_reset();
        let mut accepted = false;
        for time in (10..=220).step_by(10) {
            let samples = sensors.map(|_| {
                Some(RangeObservation {
                    distance: Length::new::<meter>(0.85),
                    timestamp: Duration::from_millis(time),
                    confidence: 1.0,
                    occluded: false,
                })
            });
            accepted |= tracker
                .update(snapshot(time), &samples)
                .unwrap()
                .distance_reset;
        }
        assert!(!accepted);
        assert_eq!(tracker.pose(), Pose::default());
    }
}
