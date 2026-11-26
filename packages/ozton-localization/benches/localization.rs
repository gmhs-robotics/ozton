//! Repeatable host throughput checks. These are not V5 latency measurements.
use std::{
    hint::black_box,
    time::{Duration, Instant},
};

use ozton_localization::{
    field::{Field, OVERRIDE},
    localizer::{
        Config, Estimator, Localizer, MotionObservations, RangeObservation, RangeSensorConfig,
        Snapshot,
    },
};
use ozton_math::{
    geometry::{BodyDelta, DVec2, Pose},
    localization::{
        mcl::{Beam, ParticleConfig, ParticleFilter},
        range::{Surface, cast, cast_masked},
    },
    position_xyzh, qty,
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

fn benchmark_override() -> Result<(), &'static str> {
    let map: Vec<_> = OVERRIDE
        .surface_map(Field::PORTABLE)
        .ok_or("invalid Override surface map")?
        .faces()
        .collect();
    let sensors = [
        position_xyzh!(0 cm, 13 cm, 92 mm, 0 deg),
        position_xyzh!(-13 cm, 0 cm, 206 mm, 90 deg),
        position_xyzh!(0 cm, -13 cm, 92 mm, 180 deg),
        position_xyzh!(13 cm, 0 cm, 206 mm, -90 deg),
    ]
    .map(|mount| RangeSensorConfig::at_mount(mount).with_visible_surfaces(&map));
    let pose = Pose::new(DVec2::new(0.25, 0.22), qty!(5 deg));
    let beams = sensors
        .into_iter()
        .map(|sensor| {
            let hit = cast_masked(pose, sensor.mount, &map, sensor.surface_mask)
                .ok_or("benchmark sensor must see a mapped surface")?;
            Ok(Beam {
                mount: sensor.mount,
                distance: qty!((hit.distance) m),
                max_range: qty!(2 m),
                surface_mask: sensor.surface_mask,
            })
        })
        .collect::<Result<Vec<_>, _>>()?;
    let mut filter = ParticleFilter::new(pose, ParticleConfig::default().with_particles(64))
        .ok_or("invalid Override MCL benchmark configuration")?;
    if filter
        .observe_position(&beams, &map, pose.heading)
        .is_none()
    {
        return Err("Override MCL benchmark requires a usable range update");
    }
    measure("Override MCL, 64 particles and four ranges", 4_000, |_| {
        black_box(filter.observe_position(black_box(&beams), black_box(&map), pose.heading));
    });
    Ok(())
}

fn main() -> Result<(), &'static str> {
    let map = [
        Surface::new(
            ozton_math::geometry::point(Length::new::<meter>(-1.8), Length::new::<meter>(-1.8)),
            ozton_math::geometry::point(Length::new::<meter>(1.8), Length::new::<meter>(-1.8)),
            ozton_math::qty!(1 m),
        ),
        Surface::new(
            ozton_math::geometry::point(Length::new::<meter>(1.8), Length::new::<meter>(-1.8)),
            ozton_math::geometry::point(Length::new::<meter>(1.8), Length::new::<meter>(1.8)),
            ozton_math::qty!(1 m),
        ),
        Surface::new(
            ozton_math::geometry::point(Length::new::<meter>(1.8), Length::new::<meter>(1.8)),
            ozton_math::geometry::point(Length::new::<meter>(-1.8), Length::new::<meter>(1.8)),
            ozton_math::qty!(1 m),
        ),
        Surface::new(
            ozton_math::geometry::point(Length::new::<meter>(-1.8), Length::new::<meter>(1.8)),
            ozton_math::geometry::point(Length::new::<meter>(-1.8), Length::new::<meter>(-1.8)),
            ozton_math::qty!(1 m),
        ),
    ];
    let sensors = [
        0.0,
        core::f64::consts::FRAC_PI_2,
        core::f64::consts::PI,
        -core::f64::consts::FRAC_PI_2,
    ]
    .map(|yaw| {
        RangeSensorConfig::new(Pose::new(
            ozton_math::geometry::point(Length::new::<meter>(0.0), Length::new::<meter>(0.0)),
            Angle::new::<radian>(yaw),
        ))
    });
    measure("four wall casts", 100_000, |tick| {
        let pose = Pose::new(
            ozton_math::geometry::point(Length::new::<meter>(0.01), Length::new::<meter>(-0.02)),
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
            maximum_gap: Duration::from_millis(100),
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
                }),
                black_box(&ranges),
                MotionObservations::default(),
                |_, _| BodyDelta::default(),
            ))
            .ok();
        });
    }
    let mut filter = ParticleFilter::new(Pose::default(), ParticleConfig::default())
        .ok_or("invalid MCL benchmark configuration")?;
    measure("MCL motion prediction", 10_000, |_| {
        filter.predict(black_box(BodyDelta {
            translation: glam::DVec2::new(0.0001, 0.001),
            heading: Angle::new::<radian>(0.001),
        }));
        black_box(&filter);
    });
    benchmark_override()
}
