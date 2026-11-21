use ozton_math::path::{Limits, Path, follower::Progress, jerryio};
use uom::si::{
    acceleration::meter_per_second_squared,
    f64::{Acceleration, Length, Time, Velocity},
    length::{inch, meter},
    time::second as time_second,
    velocity::meter_per_second,
};

#[test]
fn actual_export_uses_field_coordinates_and_real_end_point() {
    let points = jerryio::parse(include_bytes!("fixtures/jerryio-v05.txt")).unwrap();
    let path = Path::new(
        &points,
        Limits {
            track_width: Length::new::<meter>(0.3),
            linear_speed: Velocity::new::<meter_per_second>(0.8),
            wheel_speed: Velocity::new::<meter_per_second>(0.8),
            acceleration: Acceleration::new::<meter_per_second_squared>(1.0),
            deceleration: Acceleration::new::<meter_per_second_squared>(1.0),
            lateral_acceleration: Acceleration::new::<meter_per_second_squared>(1.2),
        },
    )
    .unwrap();
    assert_eq!(path.start().position.to_array(), [0.0, 0.0]);
    assert_eq!(
        path.end().position.to_array(),
        [-21.466, -11.599].map(|x| Length::new::<inch>(x).get::<meter>())
    );
    assert_eq!(path.end().speed, 0.0);
    for i in 1..20 {
        let time = path.end().time * f64::from(i) / 20.0;
        let sample = path.at_time(Time::new::<time_second>(time));
        assert!((path.at_distance(Length::new::<meter>(sample.distance)).time - time).abs() < 1e-8);
    }
}

#[test]
fn projection_keeps_the_current_branch_at_a_crossing() {
    use ozton_math::path::Waypoint;
    let path = Path::new(
        &[
            Waypoint::new(Length::new::<meter>(-1.0), Length::new::<meter>(-1.0), 1.0),
            Waypoint::new(Length::new::<meter>(1.0), Length::new::<meter>(1.0), 1.0),
            Waypoint::new(Length::new::<meter>(-1.0), Length::new::<meter>(1.0), 1.0),
            Waypoint::new(Length::new::<meter>(1.0), Length::new::<meter>(-1.0), 1.0),
        ],
        Limits {
            track_width: Length::new::<meter>(0.3),
            linear_speed: Velocity::new::<meter_per_second>(0.8),
            wheel_speed: Velocity::new::<meter_per_second>(0.8),
            acceleration: Acceleration::new::<meter_per_second_squared>(1.0),
            deceleration: Acceleration::new::<meter_per_second_squared>(1.0),
            lateral_acceleration: Acceleration::new::<meter_per_second_squared>(1.2),
        },
    )
    .unwrap();
    let mut progress = Progress::new(&path);
    for i in 1..=15 {
        let point = path
            .at_distance(Length::new::<meter>(f64::from(i) * 0.1))
            .position;
        progress.update(point, Length::new::<meter>(0.11));
    }
    assert!((progress.distance() - 1.5).abs() < 1e-10);
    assert!(progress.distance() < path.end().distance * 0.25);
}

#[test]
fn compatible_export_segments_are_profiled_through_the_junction() {
    use ozton_math::path::jerryio::{ChainError, parse_chain};

    let first = b"0,0,100\n4,0,100\n8,0,0\nendData";
    let second = b"8,0,100\n12,0,100\n16,0,0\nendData";
    let points = parse_chain(&[first, second]).unwrap();
    let path = Path::new(
        &points,
        Limits {
            track_width: Length::new::<meter>(0.3),
            linear_speed: Velocity::new::<meter_per_second>(0.8),
            wheel_speed: Velocity::new::<meter_per_second>(0.8),
            acceleration: Acceleration::new::<meter_per_second_squared>(1.0),
            deceleration: Acceleration::new::<meter_per_second_squared>(1.0),
            lateral_acceleration: Acceleration::new::<meter_per_second_squared>(1.2),
        },
    )
    .unwrap();
    assert_eq!(points.len(), 5);
    assert!(path.at_distance(Length::new::<inch>(8.0)).speed > 0.2);
    assert_eq!(path.end().speed, 0.0);
    assert_eq!(parse_chain(&[first, first]), Err(ChainError::Junction(1)));
}
