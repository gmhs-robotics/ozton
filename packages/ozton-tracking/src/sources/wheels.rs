//! Odometry from mounted tracking wheels and optional differential drive encoders.

use glam::{DVec2, DVec3};
use ozton_math::{
    geometry::{BodyDelta, direction, wrap_heading},
    localization::wheel_fit::fit_wheel_motion,
    uom::si::f64::{Angle, Length},
};

use super::{Snapshot, vex::WheelSource};
use crate::{RotarySensor, mount::Vec3, wheeled::TrackingWheel};

/// A tracking wheel at a surveyed position. Positive travel follows its bearing,
/// measured counterclockwise from robot forward.
pub struct MountedWheel {
    source: Box<dyn WheelSource>,
    projection: DVec3,
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
        }
    }
    /// Geometry of the sensor observation in the chassis frame.
    pub fn projection(&self) -> DVec3 {
        self.projection
    }

    pub fn is_valid(&self) -> bool {
        self.projection.is_finite() && self.source.valid()
    }

    /// Read cumulative travel. The localization stage computes motion between samples.
    pub fn sample(&self) -> Option<Length> {
        self.source.read_travel()
    }
}

/// How paired drive encoder readings can constrain the pose.
#[derive(Clone, Copy)]
pub enum DriveWheels {
    Forward,
    ForwardAndHeading,
}

/// Fits body movement from cumulative pod readings and fresh drive/IMU data.
/// Geometry and baselines are stored here. Sensor ownership stays with tracking.
pub struct WheelOdometry {
    projections: Vec<DVec3>,
    previous: Vec<Option<Length>>,
    travel: Vec<Option<f64>>,
}

impl WheelOdometry {
    pub fn new(projections: impl IntoIterator<Item = DVec3>) -> Self {
        let projections: Vec<_> = projections.into_iter().collect();
        let previous = vec![None; projections.len()];
        let travel = vec![None; projections.len()];
        Self {
            projections,
            previous,
            travel,
        }
    }

    pub fn observe(&mut self, positions: &[Option<Length>]) {
        for (index, (previous, travel)) in
            self.previous.iter_mut().zip(&mut self.travel).enumerate()
        {
            let position = positions
                .get(index)
                .copied()
                .flatten()
                .filter(|value| value.value.is_finite());
            *travel = std::mem::replace(previous, position)
                .zip(position)
                .map(|(before, after)| after.value - before.value)
                .filter(|value| value.is_finite());
        }
    }

    pub fn delta(
        &self,
        previous: Snapshot,
        next: Snapshot,
        drive: Option<DriveWheels>,
        track_width: Option<Length>,
        use_imu: bool,
    ) -> BodyDelta {
        let yaw = if use_imu {
            previous
                .imu
                .zip(next.imu)
                .map(|(before, after)| wrap_heading(after - before))
                .filter(|yaw| yaw.is_finite())
        } else {
            None
        };
        let pods = self
            .projections
            .iter()
            .copied()
            .zip(&self.travel)
            .filter_map(|(row, travel)| Some((row, (*travel)?)));
        if let Some(movement) = fit_wheel_motion(pods.clone(), yaw, false) {
            return movement;
        }

        let width = track_width.filter(|width| width.value.is_finite() && width.value > 0.0);
        let drive = drive_rows(previous, next, drive, width)
            .into_iter()
            .flatten()
            .map(|(row, travel)| (row * 0.5, travel * 0.5));
        let observations = pods.chain(drive);
        fit_wheel_motion(observations.clone(), yaw, false)
            .or_else(|| fit_wheel_motion(observations.clone(), yaw, true))
            .unwrap_or(BodyDelta {
                translation: DVec2::ZERO,
                heading: yaw.unwrap_or_default(),
            })
    }
}

fn drive_rows(
    previous: Snapshot,
    next: Snapshot,
    drive: Option<DriveWheels>,
    width: Option<Length>,
) -> [Option<(DVec3, f64)>; 2] {
    if drive.is_none() {
        return [None; 2];
    }
    let [left, right] = [0, 1].map(|index| {
        let travel = next.drive[index]?.value - previous.drive[index]?.value;
        travel.is_finite().then_some(travel)
    });
    if matches!(drive, Some(DriveWheels::ForwardAndHeading))
        && let Some(width) = width
    {
        return [
            left.map(|travel| (DVec3::new(0.0, 1.0, -0.5 * width.value), travel)),
            right.map(|travel| (DVec3::new(0.0, 1.0, 0.5 * width.value), travel)),
        ];
    }
    let forward = match (left, right) {
        (Some(left), Some(right)) => Some((left + right) * 0.5),
        (left, right) => left.or(right),
    };
    [forward.map(|travel| (DVec3::Y, travel)), None]
}

#[cfg(test)]
mod tests {
    use ozton_math::uom::si::{
        angle::{degree, radian},
        length::meter,
    };

    use super::*;

    fn length(value: f64) -> Length {
        Length::new::<meter>(value)
    }

    #[test]
    fn complete_pods_win_over_slipping_drive_and_drive_fills_missing_forward() {
        let mut layout = WheelOdometry::new([DVec3::Y, DVec3::X]);
        layout.observe(&[Some(length(0.0)); 2]);
        layout.observe(&[Some(length(0.05)), Some(length(0.02))]);
        let before = Snapshot {
            drive: [Some(length(0.0)); 2],
            imu: Some(Angle::new::<radian>(0.0)),
            ..Snapshot::default()
        };
        let after = Snapshot {
            drive: [Some(length(0.1)), Some(length(0.3))],
            ..before
        };
        let motion = layout.delta(before, after, Some(DriveWheels::Forward), None, true);
        assert!(motion.translation.distance(DVec2::new(0.02, 0.05)) < 1e-12);

        layout.observe(&[None, Some(length(0.05))]);
        let motion = layout.delta(before, after, Some(DriveWheels::Forward), None, true);
        assert!(motion.translation.distance(DVec2::new(0.03, 0.2)) < 1e-12);
    }

    #[test]
    fn paired_drive_uses_imu_or_wheel_yaw_without_requiring_lateral() {
        let layout = WheelOdometry::new([]);
        let before = Snapshot {
            drive: [Some(length(5.0)), Some(length(3.0))],
            imu: Some(Angle::new::<radian>(0.0)),
            ..Snapshot::default()
        };
        let after = Snapshot {
            drive: [Some(length(5.06)), Some(length(3.0 + 0.14))],
            ..before
        };
        for (imu, heading) in [(None, 0.2), (Some(2.0), 2.0)] {
            let after = Snapshot {
                imu: imu.map(Angle::new::<radian>),
                ..after
            };
            let motion = layout.delta(
                before,
                after,
                Some(DriveWheels::ForwardAndHeading),
                Some(length(0.4)),
                true,
            );
            assert!(motion.translation.distance(DVec2::new(0.0, 0.1)) < 1e-12);
            assert!((motion.heading - heading).abs() < 1e-12);
        }
    }

    #[test]
    fn skew_wheels_resolve_translation_and_yaw_without_an_imu() {
        let mounts = [(0.1, 0.0, 0.0), (0.2, 0.0, -35.0), (0.0, 0.2, 60.0)];
        let mut layout = WheelOdometry::new(mounts.map(|(x, y, angle)| {
            MountedWheel::new(
                (),
                Vec3::new(length(x), length(y), length(0.0)),
                Angle::new::<degree>(angle),
            )
            .projection()
        }));
        let baseline = [0.0, 2.5, -1.0];
        layout.observe(&baseline.map(|value| Some(length(value))));
        let travel = DVec2::new(0.02, 0.12);
        let yaw = -0.2;
        let positions = std::array::from_fn::<_, 3, _>(|index| {
            let (x, y, bearing) = mounts[index];
            let angle = Angle::new::<degree>(bearing).value;
            Some(length(
                baseline[index] + (travel.y + x * yaw) * angle.cos()
                    - (travel.x - y * yaw) * angle.sin(),
            ))
        });
        layout.observe(&positions);
        let same = Snapshot::default();
        let motion = layout.delta(same, same, None, None, false);
        assert!(motion.translation.distance(travel) < 1e-12);
        assert!((motion.heading - yaw).abs() < 1e-12);

        layout.observe(&[None, positions[1], positions[2]]);
        layout.observe(&[Some(length(200.0)), positions[1], positions[2]]);
        let motion = layout.delta(same, same, None, None, false);
        assert_eq!(motion.translation, DVec2::ZERO);
        assert_eq!(motion.heading, 0.0);
    }

    #[test]
    fn imu_and_sideways_pod_compensate_for_rotation_with_one_drive_encoder() {
        let mut layout = WheelOdometry::new([DVec3::new(1.0, 0.0, 0.1)]);
        layout.observe(&[Some(length(0.0))]);
        layout.observe(&[Some(length(0.03))]);
        let before = Snapshot {
            drive: [Some(length(0.0)); 2],
            imu: Some(Angle::new::<radian>(0.0)),
            ..Snapshot::default()
        };
        let after = Snapshot {
            drive: [Some(length(0.06)), None],
            imu: Some(Angle::new::<radian>(0.2)),
            ..before
        };
        let motion = layout.delta(
            before,
            after,
            Some(DriveWheels::ForwardAndHeading),
            Some(length(0.4)),
            true,
        );
        assert!(motion.translation.distance(DVec2::new(0.01, 0.1)) < 1e-12);
    }
}
