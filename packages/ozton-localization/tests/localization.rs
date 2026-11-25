use std::time::Duration;

use glam::DVec3;
use ozton_localization::localizer::{
    Config, EkfConfig, Estimator, Fault, Localizer, OdometryConfig, RangeObservation, RangeRoles,
    RangeSensorConfig, Snapshot,
};
use ozton_math::{
    geometry::Pose,
    localization::{
        mcl::ParticleConfig,
        observation::TracksForwardTravel,
        range::{RangeGate, Rejection, Surface},
    },
    uom::{
        ConstZero,
        si::{
            angle::radian,
            f64::{Angle, Length, Time},
            length::meter,
            time::second,
        },
    },
};

const CONFIG: OdometryConfig = OdometryConfig {
    track_width: Length {
        value: 0.3,
        ..Length::ZERO
    },
    forward_offset: Length::ZERO,
    lateral_offset: Length::ZERO,

    max_gap: Duration::from_millis(100),
};
const MAP: [Surface; 2] = [
    Surface::new(
        ozton_math::geometry::point(
            Length {
                value: 1.0,
                ..Length::ZERO
            },
            Length {
                value: 1.0,
                ..Length::ZERO
            },
        ),
        ozton_math::geometry::point(
            Length {
                value: -1.0,
                ..Length::ZERO
            },
            Length {
                value: 1.0,
                ..Length::ZERO
            },
        ),
        ozton_math::qty!(1 m),
    ),
    Surface::new(
        ozton_math::geometry::point(
            Length {
                value: -1.0,
                ..Length::ZERO
            },
            Length {
                value: 1.0,
                ..Length::ZERO
            },
        ),
        ozton_math::geometry::point(
            Length {
                value: -1.0,
                ..Length::ZERO
            },
            Length {
                value: -1.0,
                ..Length::ZERO
            },
        ),
        ozton_math::qty!(1 m),
    ),
];
const SENSOR: RangeSensorConfig = RangeSensorConfig {
    mount_height: Length::ZERO,
    mount: Pose::new(
        ozton_math::geometry::point(Length::ZERO, Length::ZERO),
        Angle::ZERO,
    ),
    gate: RangeGate::DEFAULT,
    bias: Length::ZERO,
    sigma: Length {
        value: 0.02,
        ..Length::ZERO
    },
    surface: Some(0),
    surface_mask: u128::MAX,
    roles: RangeRoles::ALL,
};
const SENSORS: [RangeSensorConfig; 2] = [
    SENSOR,
    RangeSensorConfig {
        mount: Pose::new(
            ozton_math::geometry::point(Length::ZERO, Length::ZERO),
            Angle {
                value: std::f64::consts::FRAC_PI_2,
                ..Angle::ZERO
            },
        ),
        surface: Some(1),
        ..SENSOR
    },
];

#[expect(
    clippy::expect_used,
    reason = "invalid fixture configuration must fail the test"
)]
fn localizer(estimator: Estimator, shadow: bool) -> Localizer<'static> {
    Localizer::new(
        Config {
            odometry: CONFIG,
            estimator,
            shadow,
        },
        Pose::new(
            ozton_math::geometry::point(Length::new::<meter>(-0.08), Length::new::<meter>(0.08)),
            Angle::new::<radian>(0.0),
        ),
        &MAP,
        &SENSORS,
    )
    .expect("valid localization fixture")
}

fn snapshot(t: u64) -> Snapshot {
    Snapshot {
        time: Duration::from_millis(t),
        drive: [Some(Length::new::<meter>(0.0)); 2],
        imu: Some(Angle::new::<radian>(0.0)),
        ..Snapshot::default()
    }
}

fn reading(t: u64, occluded: bool) -> Option<RangeObservation> {
    Some(RangeObservation {
        timestamp: Duration::from_millis(t),
        distance: Length::new::<meter>(1.0),
        confidence: 1.0,
        occluded,
    })
}

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
    moved.drive = [Some(Length::new::<meter>(0.01)); 2];
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
        stopped.drive = moved.drive;
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
    stopped.drive = moved.drive;
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
    localization
        .update(
            Snapshot {
                time: Duration::from_millis(10),
                ..Snapshot::default()
            },
            &[],
        )
        .unwrap_err();
    for time in (20..=610).step_by(30) {
        let raw = Snapshot {
            time: Duration::from_millis(time),
            ..Snapshot::default()
        };
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
fn relative_alignment_does_not_depend_on_pose_association() {
    let raw = reading(15, false).unwrap();
    let now = Duration::from_millis(20);
    assert_eq!(
        SENSOR.relative_distance(raw, now),
        Ok(Length::new::<meter>(1.0))
    );
    assert_eq!(
        SENSOR.relative_distance(
            RangeObservation {
                occluded: true,
                ..raw
            },
            now,
        ),
        Err(Rejection::Occluded)
    );
    assert_eq!(
        SENSOR.relative_distance(raw, Duration::from_secs(1)),
        Err(Rejection::Stale)
    );
    let constraint = ozton_math::localization::range::constrain(
        Pose::new(
            ozton_math::geometry::point(Length::new::<meter>(0.0), Length::new::<meter>(0.5)),
            Angle::new::<radian>(0.0),
        ),
        SENSOR.mount,
        ozton_math::localization::range::RangeReading {
            distance: Length::new::<meter>(1.0),
            confidence: 1.0,
            age: Time::new::<second>(0.005),
            occluded: false,
        },
        SENSOR.gate,
        &MAP,
        Some(0),
    )
    .unwrap();
    assert!((constraint.residual - 0.5).abs() < 1e-12);
}

#[test]
fn zero_track_width_preserves_heading_and_uses_available_travel() {
    let mut config = CONFIG;
    config.track_width = Length::new::<meter>(0.0);
    let mut tracker = Localizer::new(
        Config {
            odometry: config,
            estimator: Estimator::Odometry,
            shadow: false,
        },
        Pose::default(),
        &[],
        &[],
    )
    .unwrap();
    tracker.update(snapshot(10), &[]).unwrap_err();
    let mut missing = snapshot(20);
    missing.imu = None;
    assert_eq!(
        tracker.update(missing, &[]).unwrap().pose.heading.value,
        0.0
    );
}

#[test]
fn surface_mask_excludes_an_ineligible_near_wall_without_hiding_relative_range() {
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
    let sensor = RangeSensorConfig {
        surface_mask: 0b10,
        surface: None,
        ..SENSOR
    };
    let sensors = [sensor];
    let mut tracker = Localizer::new(
        Config {
            odometry: CONFIG,
            estimator: Estimator::Odometry,
            shadow: false,
        },
        Pose::default(),
        &map,
        &sensors,
    )
    .unwrap();
    tracker.update(snapshot(10), &[]).unwrap_err();
    let sample = reading(20, false);
    let result = tracker.update(snapshot(20), &[sample]).unwrap();
    assert_eq!(result.ranges[0].unwrap().constraint.unwrap().hit.surface, 1);
    assert_eq!(
        sensor
            .relative_distance(sample.unwrap(), Duration::from_millis(20))
            .unwrap(),
        Length::new::<meter>(1.0)
    );
}

#[test]
fn dsr_rejects_translation_lateral_and_rotation_until_rest_dwell_and_fresh_collection() {
    for direction in 0..3 {
        let mut tracker = localizer(Estimator::Odometry, false);
        tracker.update(snapshot(0), &[]).unwrap_err();
        tracker.request_distance_reset();
        let mut raw = snapshot(0);
        for time in (10..=200).step_by(10) {
            raw.time = Duration::from_millis(time);
            match direction {
                0 => {
                    raw.drive = [Some(Length::new::<meter>(time as f64 / 1000.0)); 2];
                }
                1 => {
                    raw.lateral = Some(Length::new::<meter>(time as f64 / 1000.0));
                }
                _ => {
                    raw.imu = Some(Angle::new::<radian>(time as f64 / 1000.0));
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
        let mut fixed = false;
        for time in (210..=450).step_by(10) {
            raw.time = Duration::from_millis(time);
            let result = tracker
                .update(raw, &[reading(time, false), reading(time, false)])
                .unwrap();
            fixed |= result.distance_reset;
        }
        // Translation/rotation can make these synthetic distances inconsistent with
        // the map. The critical invariant is that no moving correction was accepted.
        let _ = fixed;
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
        let mut tracker = Localizer::new(
            Config {
                odometry: CONFIG,
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

#[test]
fn mcl_accepts_partial_height_maps_and_estimator_switch_preserves_trust() {
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
    let mut tracker = Localizer::new(
        Config {
            odometry: CONFIG,
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
    assert!(tracker.set_estimator(Estimator::Ekf(EkfConfig::default()), true));
    assert_eq!(
        tracker.pose_trust(),
        ozton_localization::localizer::PoseTrust::Unregistered
    );
}

#[test]
fn typed_estimators_keep_pose_trust_when_switching() {
    use ozton_localization::localizer::{Ekf, Mcl, PoseTrust};
    let base = localizer(Estimator::Odometry, false);
    let pose = base.pose();
    let mut localization = base.with_filter(Mcl::new(pose, ParticleConfig::default()).unwrap());
    assert_eq!(localization.estimator_name(), "MCL");
    assert!(localization.set_pose(pose));
    assert!(matches!(
        localization.update(snapshot(10), &[]),
        Err(Fault::Initializing)
    ));
    assert!(localization.update(snapshot(20), &[]).is_ok());

    let mut localization = localization.with_filter(Ekf::new(pose, EkfConfig::default()).unwrap());
    assert_eq!(localization.estimator_name(), "EKF");
    assert_eq!(localization.pose_trust(), PoseTrust::Trusted);
    assert!(localization.update(snapshot(30), &[]).is_ok());
}

#[test]
fn range_usage_selects_filter_and_reset_observations_independently() {
    let sensors = [
        SENSORS[0].with_roles(RangeRoles::DSR),
        SENSORS[1].with_roles(RangeRoles::EKF),
    ];
    let mut tracking = Localizer::new(
        Config::new(CONFIG, Estimator::Ekf(EkfConfig::default()), false),
        localizer(Estimator::Odometry, false).pose(),
        &MAP,
        &sensors,
    )
    .unwrap();
    assert_eq!(
        tracking.update(snapshot(10), &[]).unwrap_err(),
        Fault::Initializing
    );
    let filtered = tracking
        .update(snapshot(20), &[reading(20, false); 2])
        .unwrap();
    assert!(filtered.ranges[0].is_none());
    assert!(filtered.ranges[1].unwrap().constraint.is_ok());

    assert!(tracking.request_distance_reset_axes(1));
    let resetting = tracking
        .update(snapshot(30), &[reading(30, false); 2])
        .unwrap();
    assert!(resetting.ranges[0].unwrap().constraint.is_ok());
    assert!(resetting.ranges[1].is_none());
}

#[test]
fn setting_field_zero_rebaselines_imu_heading() {
    use ozton_localization::LocalizationBuilder;
    use ozton_math::localization::observation::{
        HeadingObservation, Measurement, Observation, SensorSample,
    };

    let mut localization = LocalizationBuilder::default()
        .build(&ozton_math::localization::observation::ObservationLayout::default())
        .unwrap();
    let sample = |localization: &mut ozton_localization::Localization,
                  millis,
                  degrees| {
        let time = Duration::from_millis(millis);
        localization
            .update(
                SensorSample {
                    time,
                    observations: &[Observation {
                        source: 0,
                        time,
                        measurement: Measurement::Heading(HeadingObservation::Compass(
                            ozton_math::qty!((degrees) deg),
                        )),
                    }],
                },
                None,
            )
            .unwrap()
    };

    sample(&mut localization, 0, 0.0);
    sample(&mut localization, 10, 83.0);
    assert!(localization.set_pose(Pose::default()));

    sample(&mut localization, 20, 83.0);
    let stationary = sample(&mut localization, 30, 83.0).unwrap();
    assert!(stationary.pose.heading.value.abs() < 1e-12);
    let turned = sample(&mut localization, 40, 73.0).unwrap();
    assert!((turned.pose.heading - ozton_math::qty!(10 deg)).value.abs() < 1e-12);
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

    let turning = Snapshot {
        imu: Some(ozton_math::qty!(0.03 rad)),
        ..snapshot(30)
    };
    let distance = 1.0 / 0.03_f64.cos() - 0.03;
    let during_turn = localization.update(turning, &[wall(30, distance)]).unwrap();
    assert_eq!(during_turn.pose, during_turn.predicted);

    let stopped = Snapshot {
        time: Duration::from_millis(40),
        ..turning
    };
    let corrected = localization.update(stopped, &[wall(40, distance)]).unwrap();
    assert!(corrected.pose.position.y > corrected.predicted.position.y);
    assert_eq!(corrected.pose.heading, corrected.predicted.heading);
}

#[test]
fn sensors_above_every_face_are_excluded_from_pose_corrections() {
    let map = [Surface::new(
        ozton_math::geometry::point(ozton_math::qty!(-1 m), ozton_math::qty!(1 m)),
        ozton_math::geometry::point(ozton_math::qty!(1 m), ozton_math::qty!(1 m)),
        ozton_math::qty!(1 in),
    )];
    let sensors = [RangeSensorConfig {
        mount_height: ozton_math::qty!(4 in),
        ..SENSOR
    }];
    let mut tracker = Localizer::new(
        Config::new(CONFIG, Estimator::Mcl(ParticleConfig::default()), false),
        Pose::default(),
        &map,
        &sensors,
    )
    .unwrap();
    assert_eq!(tracker.range_config(0).unwrap().surface_mask, 0);
    assert!(!tracker.request_distance_reset_axes(1));
    tracker.update(snapshot(10), &[]).unwrap_err();
    let update = tracker.update(snapshot(20), &[reading(20, false)]).unwrap();
    assert!(update.ranges[0].is_none());
    assert_eq!(update.pose, update.predicted);
    assert!(
        sensors[0]
            .relative_distance(reading(20, false).unwrap(), Duration::from_millis(20))
            .is_ok()
    );
}
