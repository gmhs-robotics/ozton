//! Odometry from mounted tracking wheels and optional differential drive encoders.

use std::time::Duration;

use glam::{DMat2, DMat3, DVec2, DVec3};
use ozton_math::{
    geometry::{BodyDelta, direction, wrap_heading},
    uom::si::f64::{Angle, Length},
};

use super::{Fault, OdometryConfig, Snapshot, vex::WheelSource};
use crate::{RotarySensor, mount::Vec3, wheeled::TrackingWheel};

const RANK_TOLERANCE: f64 = 1e-9;

/// A tracking wheel at a surveyed position. Positive travel follows its bearing,
/// measured counterclockwise from robot forward.
pub struct MountedWheel {
    source: Box<dyn WheelSource>,
    projection: DVec3,
    previous: Option<Length>,
    travel: Option<f64>,
}

impl MountedWheel {
    /// Mount a rotary encoder on a tracking wheel. Gearing is wheel turns
    /// per encoder turn. Pass `None` for a direct drive.
    pub fn from_rotary(
        sensor: impl RotarySensor + 'static,
        diameter: Length,
        position: Vec3<Length>,
        bearing: Angle,
        gearing: Option<f64>,
    ) -> Self {
        Self::new(
            TrackingWheel::new(sensor, diameter, Length::default(), gearing),
            position,
            bearing,
        )
    }

    /// Use the right and forward coordinates relative to the tracked origin.
    /// The vertical coordinate does not affect planar odometry.
    pub fn new(source: impl WheelSource + 'static, position: Vec3<Length>, bearing: Angle) -> Self {
        let axis = direction(bearing);
        let rotation = axis.y * position.right.value - axis.x * position.forward.value;
        Self {
            source: Box::new(source),
            projection: axis.extend(rotation),
            previous: None,
            travel: None,
        }
    }
}

/// Drive travel available to the odometry fit.
#[derive(Clone, Copy)]
pub enum DriveWheels {
    /// The mean left and right distance estimates forward translation.
    Forward,
    /// The left and right distances can also estimate yaw using track width.
    ForwardAndHeading(Length),
}

/// Prepared wheel geometry. Reads each pod once per tick and fits body travel to
/// the available observations. A differential drive contributes one observation per side.
pub struct WheelOdometry {
    wheels: Vec<MountedWheel>,
    drive: Option<DriveWheels>,
    imu_heading: bool,
    forward_only: bool,
    yaw_scale: f64,
}

impl WheelOdometry {
    /// Prepare a wheel layout. Without a heading source, wheel geometry must
    /// observe yaw. Forward-aligned layouts may assume zero lateral travel.
    pub fn new(
        wheels: Vec<MountedWheel>,
        drive: Option<DriveWheels>,
        imu_heading: bool,
        require_lateral: bool,
    ) -> Option<Self> {
        if matches!(drive, Some(DriveWheels::ForwardAndHeading(width))
            if !width.value.is_finite() || width.value <= 0.0)
            || wheels
                .iter()
                .any(|wheel| !wheel.projection.is_finite() || !wheel.source.valid())
        {
            return None;
        }
        let forward_only = !require_lateral
            && wheels
                .iter()
                .all(|wheel| wheel.projection.x.abs() <= RANK_TOLERANCE);
        let mut layout = Self {
            wheels,
            drive,
            imu_heading,
            forward_only,
            yaw_scale: 1.0,
        };
        let yaw_scale = layout
            .drive_rows()
            .into_iter()
            .flatten()
            .chain(layout.wheels.iter().map(|wheel| wheel.projection))
            .map(|row| row.z.abs())
            .fold(0.0, f64::max);
        if yaw_scale > 0.0 {
            layout.yaw_scale = yaw_scale;
        }
        layout.fit(
            layout
                .drive_rows()
                .into_iter()
                .flatten()
                .chain(layout.wheels.iter().map(|wheel| wheel.projection))
                .map(|row| (row, 0.0)),
            imu_heading.then_some(0.0),
        )?;
        Some(layout)
    }

    fn drive_rows(&self) -> [Option<DVec3>; 2] {
        match self.drive {
            Some(DriveWheels::ForwardAndHeading(width)) => [
                Some(DVec3::new(0.0, 1.0, -0.5 * width.value)),
                Some(DVec3::new(0.0, 1.0, 0.5 * width.value)),
            ],
            Some(DriveWheels::Forward) => [Some(DVec3::Y), None],
            None => [None; 2],
        }
    }

    /// Whether to sample the drive model during a tracking tick.
    pub fn uses_drive(&self) -> bool {
        self.drive.is_some()
    }

    /// Whether the layout takes yaw from an IMU.
    pub fn uses_imu_heading(&self) -> bool {
        self.imu_heading
    }

    /// Capture tracking pod travel before updating the localizer for this tick.
    pub fn capture(&mut self) {
        for wheel in &mut self.wheels {
            let current = wheel.source.read_travel();
            wheel.travel = core::mem::replace(&mut wheel.previous, current)
                .zip(current)
                .map(|(before, after)| after.value - before.value);
        }
    }

    /// Resolve the local displacement and yaw over one sample interval.
    pub fn delta(
        &self,
        previous: Snapshot,
        raw: Snapshot,
        config: OdometryConfig,
        dt: Duration,
    ) -> Result<BodyDelta, Fault> {
        let seconds = dt.as_secs_f64();
        let angular_limit = config.max_angular_speed.value * seconds;
        let linear_limit = config.max_linear_speed.value * seconds;
        let imu_yaw = self
            .imu_heading
            .then_some(previous.imu.zip(raw.imu))
            .flatten()
            .map(|(before, after)| wrap_heading(after - before))
            .filter(|yaw| yaw.is_finite() && yaw.abs() <= angular_limit);
        let is_valid = |row: DVec3, travel: f64| {
            travel.is_finite() && travel.abs() <= linear_limit + row.z.abs() * angular_limit
        };
        let drive = self.drive_travel(previous, raw, is_valid);
        let pods = self.wheels.iter().filter_map(|wheel| {
            let travel = wheel.travel?;
            is_valid(wheel.projection, travel).then_some((wheel.projection, travel))
        });
        // Prefer complete pod geometry. If drive travel fills a missing axis,
        // give its rows one quarter of the weight of an independent pod.
        let motion = self.fit(pods.clone(), imu_yaw).or_else(|| {
            if drive.iter().all(Option::is_none) {
                return None;
            }
            let drive = drive
                .into_iter()
                .flatten()
                .map(|(row, travel)| (row * 0.5, travel * 0.5));
            self.fit(pods.clone().chain(drive), imu_yaw)
        });
        let movement = motion.ok_or_else(|| {
            if !pods
                .clone()
                .chain(drive.into_iter().flatten())
                .any(|(row, _)| row.y.abs() > RANK_TOLERANCE)
            {
                Fault::Forward
            } else if imu_yaw.is_none() {
                Fault::Heading
            } else {
                Fault::Lateral
            }
        })?;
        if !movement.translation.is_finite() {
            return Err(Fault::InvalidEstimate);
        }
        if !movement.heading.is_finite() || movement.heading.abs() > angular_limit {
            return Err(Fault::Heading);
        }
        let translation_limit = linear_limit + RANK_TOLERANCE;
        if movement.translation.y.abs() > translation_limit {
            return Err(Fault::Forward);
        }
        if movement.translation.x.abs() > translation_limit {
            return Err(Fault::Lateral);
        }
        Ok(movement)
    }

    fn drive_travel(
        &self,
        previous: Snapshot,
        raw: Snapshot,
        is_valid: impl Fn(DVec3, f64) -> bool,
    ) -> [Option<(DVec3, f64)>; 2] {
        let [Some(left), Some(right)] = [0, 1].map(|side| {
            let travel = raw.drive[side]?.value - previous.drive[side]?.value;
            travel.is_finite().then_some(travel)
        }) else {
            return [None; 2];
        };
        match self.drive {
            Some(DriveWheels::Forward) => {
                let mean = (left + right) * 0.5;
                [is_valid(DVec3::Y, mean).then_some((DVec3::Y, mean)), None]
            }
            Some(DriveWheels::ForwardAndHeading(_)) => {
                let rows = self.drive_rows();
                let travel = [left, right];
                let pair = [0, 1].map(|index| {
                    let row = rows[index]?;
                    is_valid(row, travel[index]).then_some((row, travel[index]))
                });
                if pair.iter().all(Option::is_some) {
                    pair
                } else {
                    [None; 2]
                }
            }
            None => [None; 2],
        }
    }

    fn fit(
        &self,
        observations: impl Iterator<Item = (DVec3, f64)>,
        yaw: Option<f64>,
    ) -> Option<BodyDelta> {
        // Accumulate the normal equations. Known yaw and forward-only geometry
        // constrain an axis directly and reduce the remaining system.
        let mut normal = DMat3::ZERO;
        let mut rhs = DVec3::ZERO;
        for (mut row, mut travel) in observations {
            if let Some(yaw) = yaw {
                travel -= row.z * yaw;
                row.z = 0.0;
            } else {
                row.z /= self.yaw_scale;
            }
            if self.forward_only {
                row.x = 0.0;
            }
            normal.x_axis += row * row.x;
            normal.y_axis += row * row.y;
            normal.z_axis += row * row.z;
            rhs += row * travel;
        }
        let solve_pair = |a: DVec2, b: DVec2, rhs: DVec2| {
            let matrix = DMat2::from_cols(a, b);
            (matrix.determinant() > RANK_TOLERANCE * a.x * b.y).then(|| matrix.inverse() * rhs)
        };
        let yy = normal.y_axis.y;
        let (translation, heading) = match (self.forward_only, yaw) {
            (true, Some(yaw)) if yy > RANK_TOLERANCE => (DVec2::new(0.0, rhs.y / yy), yaw),
            (true, Some(_)) => return None,
            (false, Some(yaw)) => (
                solve_pair(
                    normal.x_axis.truncate(),
                    normal.y_axis.truncate(),
                    rhs.truncate(),
                )?,
                yaw,
            ),
            (true, None) => {
                let yz = solve_pair(
                    DVec2::new(yy, normal.y_axis.z),
                    DVec2::new(normal.z_axis.y, normal.z_axis.z),
                    DVec2::new(rhs.y, rhs.z),
                )?;
                (DVec2::new(0.0, yz.x), yz.y / self.yaw_scale)
            }
            (false, None) => {
                if normal.determinant() <= RANK_TOLERANCE * normal.x_axis.x * yy * normal.z_axis.z {
                    return None;
                }
                let solution = normal.inverse() * rhs;
                (solution.truncate(), solution.z / self.yaw_scale)
            }
        };
        Some(BodyDelta {
            translation,
            heading,
        })
    }
}

#[cfg(test)]
mod tests {
    use std::{cell::Cell, rc::Rc};

    use ozton_math::uom::si::{
        angle::{degree, radian},
        angular_velocity::radian_per_second,
        f64::{AngularVelocity, Velocity},
        length::meter,
        velocity::meter_per_second,
    };

    use super::*;

    #[derive(Clone, Default)]
    struct Encoder(Rc<Cell<Option<Length>>>);

    impl WheelSource for Encoder {
        fn read_travel(&self) -> Option<Length> {
            self.0.get()
        }
    }

    fn length(meters: f64) -> Length {
        Length::new::<meter>(meters)
    }

    fn config() -> OdometryConfig {
        OdometryConfig::from_units(
            length(0.0),
            length(0.0),
            length(0.0),
            false,
            Velocity::new::<meter_per_second>(4.0),
            AngularVelocity::new::<radian_per_second>(10.0),
            Duration::from_secs(1),
        )
    }

    #[test]
    fn complete_pods_win_over_slipping_drive_and_drive_fills_missing_forward() {
        let forward = Encoder::default();
        let lateral = Encoder::default();
        let position = Vec3::new(length(0.0), length(0.0), length(0.0));
        let wheels = vec![
            MountedWheel::new(forward.clone(), position, Angle::new::<degree>(0.0)),
            MountedWheel::new(lateral.clone(), position, Angle::new::<degree>(-90.0)),
        ];
        let mut layout =
            WheelOdometry::new(wheels, Some(DriveWheels::Forward), true, true).unwrap();
        forward.0.set(Some(length(0.0)));
        lateral.0.set(Some(length(0.0)));
        layout.capture();
        forward.0.set(Some(length(0.05)));
        lateral.0.set(Some(length(0.02)));
        layout.capture();
        let dt = Duration::from_millis(100);
        let before = Snapshot {
            drive: [Some(length(0.0)); 2],
            imu: Some(Angle::new::<radian>(0.0)),
            ..Snapshot::default()
        };
        let after = Snapshot {
            time: dt,
            drive: [Some(length(0.1)), Some(length(0.3))],
            ..before
        };
        let motion = layout.delta(before, after, config(), dt).unwrap();
        assert!(motion.translation.distance(DVec2::new(0.02, 0.05)) < 1e-12);
        assert_eq!(motion.heading, 0.0);

        forward.0.set(None);
        lateral.0.set(Some(length(0.05)));
        layout.capture();
        let motion = layout.delta(before, after, config(), dt).unwrap();
        assert!(motion.translation.distance(DVec2::new(0.03, 0.2)) < 1e-12);
        assert!(matches!(
            layout.delta(before, Snapshot { imu: None, ..after }, config(), dt),
            Err(Fault::Heading)
        ));
    }

    #[test]
    fn full_drive_encoders_supply_yaw_when_imu_is_missing_or_implausible() {
        let layout = WheelOdometry::new(
            vec![],
            Some(DriveWheels::ForwardAndHeading(length(0.4))),
            true,
            false,
        )
        .unwrap();
        let dt = Duration::from_millis(100);
        let before = Snapshot {
            drive: [Some(length(0.0)); 2],
            imu: Some(Angle::new::<radian>(0.0)),
            ..Snapshot::default()
        };
        let after = Snapshot {
            time: dt,
            drive: [Some(length(0.06)), Some(length(0.14))],
            ..before
        };
        for (imu, expected) in [(Some(0.1), 0.1), (None, 0.2), (Some(2.0), 0.2)] {
            let sample = Snapshot {
                imu: imu.map(Angle::new::<radian>),
                ..after
            };
            let motion = layout.delta(before, sample, config(), dt).unwrap();
            assert!((motion.heading - expected).abs() < 1e-12);
            assert!((motion.translation.y - 0.1).abs() < 1e-12);
        }
    }

    #[test]
    fn skew_wheels_resolve_translation_and_yaw_without_an_imu() {
        let mounts = [(0.1, 0.0, 0.0), (0.2, 0.0, -35.0), (0.0, 0.2, 60.0)];
        let encoders = mounts.map(|_| Encoder::default());
        let wheels = mounts
            .into_iter()
            .zip(&encoders)
            .map(|((x, y, angle), encoder)| {
                MountedWheel::new(
                    encoder.clone(),
                    Vec3::new(length(x), length(y), length(0.0)),
                    Angle::new::<degree>(angle),
                )
            })
            .collect();
        let mut layout = WheelOdometry::new(wheels, None, false, true).unwrap();

        // Each encoder has a different absolute zero.
        let baselines = [0.0, 2.5, -1.0];
        for (encoder, zero) in encoders.iter().zip(baselines) {
            encoder.0.set(Some(length(zero)));
        }
        layout.capture();

        let right = 0.02;
        let forward = 0.12;
        let yaw = -0.2;
        for (((x, y, angle), encoder), zero) in mounts.into_iter().zip(&encoders).zip(baselines) {
            let angle = Angle::new::<degree>(angle).value;
            let travel = (forward + x * yaw) * angle.cos() - (right - y * yaw) * angle.sin();
            encoder.0.set(Some(length(zero + travel)));
        }
        layout.capture();
        let dt = Duration::from_millis(100);
        let raw = Snapshot {
            time: dt,
            ..Snapshot::default()
        };
        let motion = layout
            .delta(Snapshot::default(), raw, config(), dt)
            .unwrap();
        assert!(motion.translation.distance(DVec2::new(right, forward)) < 1e-12);
        assert!((motion.heading - yaw).abs() < 1e-12);

        // A returning encoder needs a fresh baseline before contributing.
        encoders[0].0.set(None);
        layout.capture();
        assert!(
            layout
                .delta(Snapshot::default(), raw, config(), dt)
                .is_err()
        );
        encoders[0].0.set(Some(length(200.0)));
        layout.capture();
        assert!(
            layout
                .delta(Snapshot::default(), raw, config(), dt)
                .is_err()
        );
    }

    #[test]
    fn paired_drive_travel_averages_forward_and_yields_wheel_heading() {
        let layout = WheelOdometry::new(
            vec![],
            Some(DriveWheels::ForwardAndHeading(length(0.4))),
            false,
            false,
        )
        .unwrap();
        let dt = Duration::from_millis(100);
        let before = Snapshot {
            drive: [Some(length(5.0)), Some(length(3.0))],
            ..Snapshot::default()
        };
        let after = Snapshot {
            time: dt,
            drive: [Some(length(5.06)), Some(length(3.14))],
            ..before
        };
        let motion = layout.delta(before, after, config(), dt).unwrap();
        assert!(motion.translation.distance(DVec2::new(0.0, 0.1)) < 1e-12);
        assert!((motion.heading - 0.2).abs() < 1e-12);
        assert!(
            WheelOdometry::new(
                vec![],
                Some(DriveWheels::ForwardAndHeading(length(0.4))),
                true,
                true
            )
            .is_none()
        );
    }

    #[test]
    fn imu_and_drive_with_sideways_pod_remove_rotation_from_travel() {
        let encoder = Encoder::default();
        let wheel = MountedWheel::new(
            encoder.clone(),
            Vec3::new(length(0.03), length(-0.1), length(0.0)),
            Angle::new::<degree>(-90.0),
        );
        let mut layout = WheelOdometry::new(
            vec![wheel],
            Some(DriveWheels::ForwardAndHeading(length(0.4))),
            true,
            true,
        )
        .unwrap();
        encoder.0.set(Some(length(0.0)));
        layout.capture();
        encoder.0.set(Some(length(0.03)));
        layout.capture();
        let dt = Duration::from_millis(100);
        let before = Snapshot {
            drive: [Some(length(0.0)); 2],
            imu: Some(Angle::new::<radian>(0.0)),
            ..Snapshot::default()
        };
        let after = Snapshot {
            time: dt,
            drive: [Some(length(0.06)), Some(length(0.14))],
            imu: Some(Angle::new::<radian>(0.2)),
            ..before
        };
        let motion = layout.delta(before, after, config(), dt).unwrap();
        assert!(motion.translation.distance(DVec2::new(0.01, 0.1)) < 1e-12);

        let missing_drive = Snapshot {
            drive: [after.drive[0], None],
            ..after
        };
        assert!(matches!(
            layout.delta(before, missing_drive, config(), dt),
            Err(Fault::Forward)
        ));
    }
}
