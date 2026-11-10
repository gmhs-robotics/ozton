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
    Surface::new(1.0, -1.0, 1.0, 1.0),
    Surface::new(1.0, 1.0, -1.0, 1.0),
];
const SENSOR: RangeSensorConfig = RangeSensorConfig {
    mount: Mount::new(0.0, 0.0, 0.0),
    gate: RangeGate::DEFAULT,
    bias: 0.0,
    sigma: 0.02,
    surface: Some(0),
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
        Pose::new(0.08, 0.08, 0.0),
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
    let fixed = localizer
        .update(snapshot(30), &[reading(19, false), reading(29, false)])
        .unwrap();
    assert!(fixed.distance_reset);
    assert!(fixed.pose.position.length() < 1e-10);
    assert_eq!(fixed.forward_velocity, 0.0);
    assert_eq!(localizer.forward_travel(), 0.0);
    let duplicate = localizer
        .update(snapshot(40), &[reading(19, false), reading(29, false)])
        .unwrap();
    assert_eq!(
        duplicate.ranges[0].unwrap().constraint,
        Err(Rejection::Duplicate)
    );
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
            Pose::new(0.50, 0.0, 0.0),
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
