use super::*;

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
fn range_usage_selects_filter_and_reset_observations_independently() {
    let sensors = [
        SENSORS[0].with_roles(RangeRoles::DSR),
        SENSORS[1].with_roles(RangeRoles::EKF),
    ];
    let mut tracking = Fixture::new(
        Config::new(MAXIMUM_GAP, Estimator::Ekf(EkfConfig::default()), false),
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
    use ozton_math::localization::observation::{
        HeadingObservation, Measurement, Observation, SensorSample,
    };

    use crate::LocalizationBuilder;

    let mut localization = LocalizationBuilder::default()
        .build(&ozton_math::localization::observation::ObservationLayout::default())
        .unwrap();
    let sample = |localization: &mut crate::Localization, millis, degrees| {
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
    let mut tracker = Fixture::new(
        Config::new(
            MAXIMUM_GAP,
            Estimator::Mcl(ParticleConfig::default()),
            false,
        ),
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
