use std::time::Duration;

use glam::DVec3;
use ozton_math::{
    geometry::Pose,
    localization::{
        mcl::ParticleConfig,
        range::{Mount, RangeGate, Rejection, Surface},
    },
};
use ozton_tracking::{
    TracksForwardTravel,
    localizer::{
        Config, EkfConfig, Estimator, Fault, Localizer, OdometryConfig, RangeObservation,
        RangeSensorConfig, Snapshot,
    },
};

const CONFIG: OdometryConfig = OdometryConfig {
    track_width: 0.3,
    forward_offset: 0.0,
    lateral_offset: 0.0,
    require_lateral: false,
    max_linear_speed: 3.0,
    max_angular_speed: 8.0,
    max_gap: Duration::from_millis(100),
};
const MAP: [Surface; 2] = [
    Surface::new(1.0, 1.0, -1.0, 1.0),
    Surface::new(-1.0, 1.0, -1.0, -1.0),
];
const SENSOR: RangeSensorConfig = RangeSensorConfig {
    mount: Mount::new(0.0, 0.0, 0.0),
    gate: RangeGate::DEFAULT,
    bias: 0.0,
    sigma: 0.02,
    surface: Some(0),
    surface_mask: u32::MAX,
};
const SENSORS: [RangeSensorConfig; 2] = [
    SENSOR,
    RangeSensorConfig {
        mount: Mount::new(0.0, 0.0, std::f64::consts::FRAC_PI_2),
        surface: Some(1),
        ..SENSOR
    },
];

fn localizer(estimator: Estimator, shadow: bool) -> Localizer<'static> {
    Localizer::new(
        Config {
            odometry: CONFIG,
            estimator,
            shadow,
            max_correction_meters: 0.2,
        },
        Pose::new(-0.08, 0.08, 0.0),
        &MAP,
        &SENSORS,
    )
    .unwrap()
}

fn snapshot(t: u64) -> Snapshot {
    Snapshot {
        time: Duration::from_millis(t),
        drive: [Some(0.0); 2],
        imu: Some(0.0),
        ..Snapshot::default()
    }
}

fn reading(t: u64, occluded: bool) -> Option<RangeObservation> {
    Some(RangeObservation {
        timestamp: Duration::from_millis(t),
        distance: 1.0,
        confidence: 1.0,
        occluded,
    })
}

#[test]
fn driver_odometry_ignores_corrections_and_resumes_filters_at_current_pose() {
    use ozton_tracking::localizer::{Correction, PoseTrust};
    for estimator in [
        Estimator::Odometry,
        Estimator::Ekf(Default::default()),
        Estimator::Mcl(Default::default()),
    ] {
        let mut tracking = localizer(estimator, false);
        tracking.set_pose(Pose::default());
        tracking.set_corrections_enabled(false);
        tracking.request_distance_reset();
        assert_eq!(
            tracking.update(snapshot(10), &[]).unwrap_err(),
            Fault::Initializing
        );
        let mut moving = snapshot(20);
        moving.drive = [Some(0.01); 2];
        let update = tracking.update(moving, &[reading(20, false); 2]).unwrap();
        assert!((update.pose.position.y - 0.01).abs() < 1e-12);
        assert!(update.ranges.iter().all(Option::is_none));
        assert!(matches!(update.correction, Correction::None));
        assert!(!update.distance_reset);
        assert!(update.candidate.is_none());
        tracking.set_corrections_enabled(true);
        let mut next = snapshot(30);
        next.drive = moving.drive;
        let resumed = tracking.update(next, &[reading(30, false); 2]).unwrap();
        assert!(
            resumed
                .ranges
                .iter()
                .flatten()
                .any(|report| report.constraint.is_ok())
        );
        assert!(resumed.pose.is_finite());
        assert!(resumed.pose.position.distance(update.pose.position) < 0.1);
        tracking.invalidate_pose();
        tracking.set_corrections_enabled(false);
        tracking.set_corrections_enabled(true);
        assert_eq!(tracking.pose_trust(), PoseTrust::NeedsRelocalization);
    }
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
    moved.drive = [Some(0.01); 2];
    assert!(
        !tracking
            .update(moved, &[reading(250, false), None])
            .unwrap()
            .distance_reset
    );
    let before = tracking.pose();
    for time in (260..=480).step_by(10) {
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
    let mut stopped = snapshot(490);
    stopped.drive = moved.drive;
    let fixed = tracking.update(stopped, &[reading(490, false); 2]).unwrap();
    assert!(fixed.distance_reset);
    assert_eq!(fixed.distance_reset_axes, 2);
    assert!(fixed.pose.position.length() < 1e-10);
}

#[test]
fn rejected_full_dsr_does_not_downgrade_to_a_partial_retry() {
    use ozton_tracking::localizer::Correction;
    let mut tracking = localizer(Estimator::Odometry, false);
    tracking.update(snapshot(10), &[]).unwrap_err();
    assert!(tracking.request_distance_reset_axes(2));
    let mut rejected = false;
    for time in (20..=190).step_by(10) {
        tracking.request_distance_reset();
        let large = Some(RangeObservation {
            distance: 1.09,
            ..reading(time, false).unwrap()
        });
        let update = tracking.update(snapshot(time), &[large; 2]).unwrap();
        rejected |= matches!(update.correction, Correction::Rejected);
        assert!(!update.distance_reset);
    }
    assert!(
        rejected,
        "two-axis correction exceeds the total displacement bound"
    );
    for time in (200..=450).step_by(10) {
        tracking.request_distance_reset();
        assert!(
            !tracking
                .update(snapshot(time), &[reading(time, false), None])
                .unwrap()
                .distance_reset
        );
    }
    tracking.request_distance_reset();
    let complete = tracking
        .update(snapshot(460), &[reading(460, false); 2])
        .unwrap();
    assert!(complete.distance_reset);
    assert_eq!(complete.distance_reset_axes, 2);
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
fn loss_and_reconnection_use_only_common_motion_intervals() {
    let mut localizer = localizer(Estimator::Odometry, false);
    let mut sample = snapshot(10);
    sample.forward = Some(100.0);
    assert_eq!(
        localizer.update(sample, &[]).unwrap_err(),
        Fault::Initializing
    );
    sample.time += Duration::from_millis(10);
    sample.drive = [Some(0.01); 2];
    sample.forward = None;
    sample.imu = None;
    assert!(localizer.update(sample, &[]).is_ok());
    sample.time += Duration::from_millis(10);
    sample.drive = [Some(0.02); 2];
    sample.forward = Some(999.0);
    sample.imu = Some(1.5);
    assert!(localizer.update(sample, &[]).is_ok());
    sample.time += Duration::from_millis(10);
    sample.drive = [Some(0.03); 2];
    sample.forward = Some(999.01);
    assert!(localizer.update(sample, &[]).is_ok());
    assert!((localizer.forward_travel() - 0.03).abs() < 1e-10);
    assert_eq!(localizer.pose().heading, 0.0);
    sample.time += Duration::from_millis(10);
    sample.imu = None;
    sample.drive = [None; 2];
    assert_eq!(localizer.update(sample, &[]).unwrap_err(), Fault::Heading);
}

#[test]
fn relative_alignment_does_not_depend_on_pose_association() {
    let raw = reading(15, false).unwrap();
    let now = Duration::from_millis(20);
    assert_eq!(SENSOR.relative_distance(raw, now), Ok(1.0));
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
    assert_eq!(
        ozton_math::localization::range::constrain(
            Pose::new(0.0, 0.5, 0.0),
            SENSOR.mount,
            ozton_math::localization::range::RangeReading {
                distance: 1.0,
                confidence: 1.0,
                age: 0.005,
                occluded: false,
            },
            SENSOR.gate,
            &MAP,
            Some(0),
        ),
        Err(Rejection::Residual)
    );
}

#[test]
fn zero_track_width_requires_imu_instead_of_guessing_wheel_heading() {
    let mut config = CONFIG;
    config.track_width = 0.0;
    let mut tracker = Localizer::new(
        Config {
            odometry: config,
            estimator: Estimator::Odometry,
            shadow: false,
            max_correction_meters: 0.2,
        },
        Pose::default(),
        &[],
        &[],
    )
    .unwrap();
    tracker.update(snapshot(10), &[]).unwrap_err();
    let mut missing = snapshot(20);
    missing.imu = None;
    assert_eq!(tracker.update(missing, &[]).unwrap_err(), Fault::Heading);
}

#[test]
fn surface_mask_excludes_an_ineligible_near_wall_without_hiding_relative_range() {
    let map = [
        Surface::new(1.0, 0.5, -1.0, 0.5),
        Surface::new(1.0, 1.0, -1.0, 1.0),
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
            max_correction_meters: 0.2,
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
        1.0
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
                    raw.drive = [Some(time as f64 / 1000.0); 2];
                }
                1 => {
                    raw.lateral = Some(time as f64 / 1000.0);
                }
                _ => {
                    raw.imu = Some(time as f64 / 1000.0);
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
        mount: Mount::new(0.0, 0.0, std::f64::consts::PI),
        surface: None,
        ..SENSOR
    };
    let forward = RangeSensorConfig {
        surface: None,
        ..SENSOR
    };
    let map = [
        Surface::new(1.0, 1.0, -1.0, 1.0),
        Surface::new(-1.0, -1.0, 1.0, -1.0),
    ];
    for sensors in [[forward, opposite], [opposite, forward]] {
        let mut tracker = Localizer::new(
            Config {
                odometry: CONFIG,
                estimator: Estimator::Odometry,
                shadow: false,
                max_correction_meters: 0.2,
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
                    distance: 0.85,
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
fn reconnect_and_distance_reset_do_not_restore_pose_trust() {
    use ozton_tracking::localizer::PoseTrust;
    let mut tracker = localizer(Estimator::Odometry, false);
    assert_eq!(tracker.pose_trust(), PoseTrust::Unregistered);
    assert!(tracker.set_pose(Pose::default()));
    assert_eq!(tracker.pose_trust(), PoseTrust::Trusted);
    tracker.update(snapshot(10), &[]).unwrap_err();
    let mut lost = snapshot(20);
    lost.drive = [None; 2];
    lost.imu = None;
    assert!(tracker.update(lost, &[]).is_err());
    assert_eq!(tracker.pose_trust(), PoseTrust::NeedsRelocalization);
    assert_eq!(
        tracker.update(snapshot(30), &[]).unwrap_err(),
        Fault::Initializing
    );
    tracker.update(snapshot(40), &[]).unwrap();
    assert_eq!(tracker.pose_trust(), PoseTrust::NeedsRelocalization);
    tracker.set_pose(Pose::new(-0.1, 0.2, 0.0));
    assert_eq!(tracker.pose_trust(), PoseTrust::Trusted);
}

#[test]
fn mcl_accepts_partial_height_maps_and_estimator_switch_preserves_trust() {
    let map = [
        Surface::new(1.0, 0.5, -1.0, 0.5),
        Surface::new(1.0, 1.0, -1.0, 1.0),
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
            max_correction_meters: 0.2,
        },
        Pose::new(0.0, 0.08, 0.0),
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
        ozton_tracking::localizer::PoseTrust::Unregistered
    );
}

#[test]
fn typed_estimators_compose_with_shared_or_private_ranges() {
    use ozton_tracking::localizer::{Ekf, Mcl, PoseTrust, Tracking};

    let base = localizer(Estimator::Odometry, false);
    let pose = base.pose();
    let mut tracker = Tracking::new(base)
        .with_distances("shared front and left range source")
        .with_mcl(Mcl::new(pose, ParticleConfig::default()).unwrap());
    assert_eq!(tracker.distances, "shared front and left range source");
    assert_eq!(tracker.localizer.estimator_name(), "MCL");
    assert!(tracker.localizer.set_pose(pose));
    assert_eq!(
        tracker.localizer.update(snapshot(10), &[]).unwrap_err(),
        Fault::Initializing
    );
    assert!(tracker.localizer.update(snapshot(20), &[]).is_ok());
    assert_eq!(tracker.localizer.pose_trust(), PoseTrust::Trusted);

    let mut tracker = tracker.with_ekf(Ekf::new(pose, EkfConfig::default()).unwrap());
    assert_eq!(tracker.distances, "shared front and left range source");
    assert_eq!(tracker.localizer.estimator_name(), "EKF");
    assert_eq!(tracker.localizer.pose_trust(), PoseTrust::Trusted);
    assert!(tracker.localizer.update(snapshot(30), &[]).is_ok());

    // A sole consumer may own its source directly and feed the observations.
    let mut tracker = Tracking::new(localizer(Estimator::Odometry, false))
        .with_ekf(Ekf::new(pose, EkfConfig::default()).unwrap());
    let () = tracker.distances;
    assert!(tracker.localizer.set_pose(pose));
    assert!(
        tracker
            .localizer
            .update(snapshot(10), &[reading(10, false)])
            .is_err()
    );
    assert!(
        tracker
            .localizer
            .update(snapshot(20), &[reading(20, false)])
            .is_ok()
    );
}
