use std::time::Duration;

use glam::DVec3;
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

use crate::localizer::{
    Config, EkfConfig, Estimator, Fault, Localizer, RangeObservation, RangeRoles,
    RangeSensorConfig, Snapshot, Update,
};

const MAXIMUM_GAP: Duration = Duration::from_millis(100);
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

fn localizer(estimator: Estimator, shadow: bool) -> Fixture<'static> {
    Fixture::new(
        Config {
            maximum_gap: MAXIMUM_GAP,
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

// Localization tests use the same wheel fit as the deployed observation pipeline.
struct Fixture<'a> {
    localizer: Localizer<'a>,
    wheels: crate::localization::wheel_odometry::WheelOdometry,
}

impl<'a> Fixture<'a> {
    fn new(
        config: Config,
        pose: Pose,
        map: &'a [Surface],
        sensors: &'a [RangeSensorConfig],
    ) -> Option<Self> {
        Some(Self {
            localizer: Localizer::new(config, pose, map, sensors)?,
            wheels: crate::localization::wheel_odometry::WheelOdometry::new([DVec3::X]),
        })
    }

    fn update(
        &mut self,
        frame: Frame,
        ranges: &[Option<RangeObservation>],
    ) -> Result<Update, Fault> {
        use crate::localization::wheel_odometry::DriveWheels;
        self.wheels.observe(&[frame.lateral]);
        self.localizer.update(
            frame.raw,
            ranges,
            crate::MotionObservations::default(),
            |before, after| {
                self.wheels.delta(
                    before,
                    after,
                    Some(DriveWheels::ForwardAndHeading),
                    Some(ozton_math::qty!(0.3 m)),
                    true,
                )
            },
        )
    }
}

impl<'a> std::ops::Deref for Fixture<'a> {
    type Target = Localizer<'a>;

    fn deref(&self) -> &Self::Target {
        &self.localizer
    }
}

impl std::ops::DerefMut for Fixture<'_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.localizer
    }
}

#[derive(Clone, Copy)]
struct Frame {
    raw: Snapshot,
    lateral: Option<Length>,
}

fn snapshot(t: u64) -> Frame {
    Frame {
        raw: Snapshot {
            time: Duration::from_millis(t),
            drive: [Some(Length::new::<meter>(0.0)); 2],
            imu: Some(Angle::new::<radian>(0.0)),
        },
        lateral: None,
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

mod estimators;
mod observations;
mod reset;
