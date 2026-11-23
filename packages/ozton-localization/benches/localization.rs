//! Repeatable host throughput checks. These are not V5 latency measurements.
use std::{
    hint::black_box,
    time::{Duration, Instant},
};

use ozton_localization::localizer::{
    Config, Estimator, Localizer, OdometryConfig, RangeObservation, RangeSensorConfig, Snapshot,
};
use ozton_math::{
    geometry::{BodyDelta, Pose},
    localization::{
        mcl::{ParticleConfig, ParticleFilter},
        range::{Mount, Surface, cast},
    },
    uom::si::{
        angle::radian,
        f64::{Angle, Length},
        length::meter,
    },
};

fn measure(name: &str, count: usize, mut step: impl FnMut(usize)) {
    let mut timings = [0.0; 5];
    for timing in &mut timings {
        let start = Instant::now();
        for tick in 0..count {
            step(black_box(tick));
        }
        *timing = start.elapsed().as_secs_f64() * 1e9 / count as f64;
    }
    timings.sort_by(f64::total_cmp);
    println!(
        "{name}: {:.1} ns/tick (median of 5, {count} ticks)",
        timings[2]
    );
}

fn main() -> Result<(), &'static str> {
    let map = [
        Surface::new(
            Length::new::<meter>(-1.8),
            Length::new::<meter>(-1.8),
            Length::new::<meter>(1.8),
            Length::new::<meter>(-1.8),
        ),
        Surface::new(
            Length::new::<meter>(1.8),
            Length::new::<meter>(-1.8),
            Length::new::<meter>(1.8),
            Length::new::<meter>(1.8),
        ),
        Surface::new(
            Length::new::<meter>(1.8),
            Length::new::<meter>(1.8),
            Length::new::<meter>(-1.8),
            Length::new::<meter>(1.8),
        ),
        Surface::new(
            Length::new::<meter>(-1.8),
            Length::new::<meter>(1.8),
            Length::new::<meter>(-1.8),
            Length::new::<meter>(-1.8),
        ),
    ];
    let sensors = [
        0.0,
        core::f64::consts::FRAC_PI_2,
        core::f64::consts::PI,
        -core::f64::consts::FRAC_PI_2,
    ]
    .map(|yaw| {
        RangeSensorConfig::new(Mount::new(
            Length::new::<meter>(0.0),
            Length::new::<meter>(0.0),
            Angle::new::<radian>(yaw),
        ))
    });
    measure("four wall casts", 100_000, |tick| {
        let pose = Pose::new(
            Length::new::<meter>(0.01),
            Length::new::<meter>(-0.02),
            Angle::new::<radian>((tick % 100) as f64 * 0.001),
        );
        for sensor in sensors {
            black_box(cast(black_box(pose), sensor.mount, black_box(&map)));
        }
    });
    for (name, estimator, count) in [
        (
            "driver odometry (MCL paused)",
            Estimator::Mcl(Default::default()),
            30_000,
        ),
        ("odometry + four ranges", Estimator::Odometry, 30_000),
        (
            "EKF + four ranges",
            Estimator::Ekf(Default::default()),
            30_000,
        ),
        (
            "MCL + four ranges",
            Estimator::Mcl(Default::default()),
            3_000,
        ),
    ] {
        let config = Config {
            odometry: OdometryConfig {
                track_width: Length::new::<meter>(0.3),
                forward_offset: Length::new::<meter>(0.0),
                lateral_offset: Length::new::<meter>(0.0),

                max_gap: Duration::from_millis(100),
            },
            estimator,
            shadow: false,
        };
        let mut localizer = Localizer::new(config, Pose::default(), &map, &sensors)
            .ok_or("invalid localizer benchmark configuration")?;
        if name.starts_with("driver") {
            localizer.set_corrections_enabled(false);
        }
        let mut sequence = 0;
        measure(name, count, |_| {
            sequence += 1;
            let time = Duration::from_millis(sequence * 10);
            let packet = Duration::from_millis((sequence / 4) * 40);
            let ranges = [Some(RangeObservation {
                timestamp: packet,
                distance: Length::new::<meter>(1.8),
                confidence: 1.0,
                occluded: false,
            }); 4];
            black_box(localizer.update(
                black_box(Snapshot {
                    time,
                    drive: [Some(Length::new::<meter>(0.0)); 2],
                    imu: Some(Angle::new::<radian>(0.0)),
                    ..Default::default()
                }),
                black_box(&ranges),
            ))
            .ok();
        });
    }
    let mut filter = ParticleFilter::new(Pose::default(), ParticleConfig::default())
        .ok_or("invalid MCL benchmark configuration")?;
    measure("MCL motion prediction", 10_000, |_| {
        filter.predict(black_box(BodyDelta {
            translation: glam::DVec2::new(0.0001, 0.001),
            heading: 0.001,
        }));
        black_box(&filter);
    });
    Ok(())
}
