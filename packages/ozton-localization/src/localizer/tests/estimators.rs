use super::*;

#[test]
fn both_estimators_improve_a_known_bias_and_shadow_never_moves_the_output() {
    for estimator in [
        Estimator::Ekf(EkfConfig {
            initial_variance: DVec3::new(0.01, 0.01, 0.001),
            ..EkfConfig::default()
        }),
        Estimator::Mcl(ParticleConfig::default()),
    ] {
        for shadow in [false, true] {
            let mut localizer = localizer(estimator, shadow);
            localizer.update(snapshot(10), &[]).unwrap_err();
            let mut update = localizer
                .update(snapshot(20), &[reading(19, false), reading(19, false)])
                .unwrap();
            let baseline = update.odometry.position.length();
            for time in [40, 60, 80] {
                update = localizer
                    .update(
                        snapshot(time),
                        &[reading(time - 1, false), reading(time - 1, false)],
                    )
                    .unwrap();
            }
            let candidate = update.candidate.unwrap().position.length();
            assert!(
                candidate < baseline * 0.6,
                "{estimator:?}: {candidate} >= {baseline}"
            );
            if shadow {
                assert_eq!(update.pose, update.odometry);
            } else {
                assert!(update.pose.position.length() < baseline * 0.6);
            }
            let blocked = localizer
                .update(snapshot(100), &[reading(99, true), reading(99, true)])
                .unwrap();
            assert_eq!(
                blocked.ranges[0].unwrap().constraint,
                Err(Rejection::Occluded)
            );
            assert_eq!(blocked.pose, update.pose);
        }
    }
}

#[test]
fn mcl_corrects_lateral_error_within_the_motion_prior() {
    let mut localization = localizer(Estimator::Mcl(ParticleConfig::default()), false);
    assert!(localization.set_pose(Pose::default()));
    localization.update(snapshot(10), &[]).unwrap_err();
    for time in (20..=610).step_by(30) {
        let raw = snapshot(time);
        let left = Some(RangeObservation {
            distance: Length::new::<meter>(0.92),
            ..reading(time - 1, false).unwrap()
        });
        localization
            .update(raw, &[reading(time - 1, false), left])
            .unwrap();
    }
    assert!(
        (localization.pose().position.x + 0.08).abs() < 0.06,
        "{:?}",
        localization.pose()
    );
    assert!(localization.pose().position.y.abs() < 0.12);
}

#[test]
fn mcl_accepts_partial_height_maps() {
    let map = [
        Surface::new(
            ozton_math::geometry::point(Length::new::<meter>(1.0), Length::new::<meter>(0.5)),
            ozton_math::geometry::point(Length::new::<meter>(-1.0), Length::new::<meter>(0.5)),
            ozton_math::qty!(1 m),
        ),
        Surface::new(
            ozton_math::geometry::point(Length::new::<meter>(1.0), Length::new::<meter>(1.0)),
            ozton_math::geometry::point(Length::new::<meter>(-1.0), Length::new::<meter>(1.0)),
            ozton_math::qty!(1 m),
        ),
    ];
    let sensors = [RangeSensorConfig {
        surface_mask: 0b10,
        surface: None,
        ..SENSOR
    }];
    let mut tracker = Fixture::new(
        Config {
            maximum_gap: MAXIMUM_GAP,
            estimator: Estimator::Mcl(ParticleConfig::default()),
            shadow: false,
        },
        Pose::new(
            ozton_math::geometry::point(Length::new::<meter>(0.0), Length::new::<meter>(0.08)),
            Angle::new::<radian>(0.0),
        ),
        &map,
        &sensors,
    )
    .unwrap();
    tracker.update(snapshot(10), &[]).unwrap_err();
    for time in (20..=100).step_by(20) {
        tracker
            .update(snapshot(time), &[reading(time, false)])
            .unwrap();
    }
    assert!(tracker.pose().position.y.abs() < 0.06);
}

#[test]
fn typed_estimators_keep_pose_trust_when_switching() {
    use crate::localizer::{Ekf, Mcl, PoseTrust};
    let base = localizer(Estimator::Odometry, false);
    let pose = base.pose();
    let mut localization = base
        .localizer
        .with_filter(Mcl::new(pose, ParticleConfig::default()).unwrap());
    assert_eq!(localization.estimator_name(), "MCL");
    assert!(localization.set_pose(pose));
    assert!(matches!(
        localization.update(
            snapshot(10).raw,
            &[],
            crate::MotionObservations::default(),
            |_, _| ozton_math::geometry::BodyDelta::default()
        ),
        Err(Fault::Initializing)
    ));
    assert!(
        localization
            .update(
                snapshot(20).raw,
                &[],
                crate::MotionObservations::default(),
                |_, _| ozton_math::geometry::BodyDelta::default()
            )
            .is_ok()
    );

    let mut localization = localization.with_filter(Ekf::new(pose, EkfConfig::default()).unwrap());
    assert_eq!(localization.estimator_name(), "EKF");
    assert_eq!(localization.pose_trust(), PoseTrust::Trusted);
    assert!(
        localization
            .update(
                snapshot(30).raw,
                &[],
                crate::MotionObservations::default(),
                |_, _| ozton_math::geometry::BodyDelta::default()
            )
            .is_ok()
    );
}

#[test]
fn mcl_filters_large_wall_residuals_and_resumes_after_a_fast_turn() {
    let mut localization = localizer(
        Estimator::Mcl(ParticleConfig {
            max_range_residual: Some(ozton_math::qty!(0.05 m)),
            max_correction_yaw_rate: Some(ozton_math::qty!(0.35 rad/s)),
            ..ParticleConfig::default()
        }),
        false,
    );
    assert!(localization.set_pose(Pose::default()));
    assert_eq!(
        localization.update(snapshot(10), &[]).unwrap_err(),
        Fault::Initializing
    );

    let wall = |millis, meters| {
        Some(RangeObservation {
            distance: ozton_math::qty!((meters) m),
            ..reading(millis, false).unwrap()
        })
    };
    let outlier = localization
        .update(snapshot(20), &[wall(20, 0.9128)])
        .unwrap();
    assert_eq!(outlier.pose, outlier.predicted);

    let mut turning = snapshot(30);
    turning.raw.imu = Some(ozton_math::qty!(0.03 rad));
    let distance = 1.0 / 0.03_f64.cos() - 0.03;
    let during_turn = localization.update(turning, &[wall(30, distance)]).unwrap();
    assert_eq!(during_turn.pose, during_turn.predicted);

    let mut stopped = turning;
    stopped.raw.time = Duration::from_millis(40);
    let corrected = localization.update(stopped, &[wall(40, distance)]).unwrap();
    assert!(corrected.pose.position.y > corrected.predicted.position.y);
    assert_eq!(corrected.pose.heading, corrected.predicted.heading);
}
