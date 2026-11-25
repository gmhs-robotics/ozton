//! IMU calibrated once before competition.

use std::time::Duration;

use glam::{DVec2, DVec3};
use ozton_diagnostics::log_event;
use ozton_math::{
    geometry::compass_heading,
    localization::observation::{ReadHeading, ReadTilt},
    uom::si::f64::{Angle, Length},
};
use vexide::{prelude::InertialSensor, smart::SmartDevice};

pub use super::Orientation;
use crate::mount::Vec3;

/// An IMU mounted flat, with a counterclockwise bearing from chassis forward.
/// Heading is independent of the IMU offset; acceleration is transported to
/// the tracked chassis origin using the surveyed lever arm.
pub struct InertialSensorTracking {
    sensor: InertialSensor,
    bearing: Angle,
    position: Vec3<Length>,
    last: Option<Orientation>,
    planar_acceleration: bool,
    previous_rotation: Option<(Duration, DVec3)>,
}

impl InertialSensorTracking {
    /// Mount the sensor at a surveyed XYZ position and heading.
    pub fn at_mount(sensor: InertialSensor, (position, bearing): (Vec3<Length>, Angle)) -> Self {
        Self {
            sensor,
            bearing,
            position,
            last: None,
            planar_acceleration: false,
            previous_rotation: None,
        }
    }

    /// Enable calibrated planar acceleration at the tracked chassis origin.
    /// The surveyed XYZ mount compensates for angular acceleration and
    /// centripetal acceleration. Verify the sensor axes and gravity
    /// compensation on the physical robot before enabling this input.
    pub fn with_planar_acceleration(mut self, enabled: bool) -> Self {
        self.planar_acceleration = enabled;
        self
    }

    pub fn position(&self) -> Vec3<Length> {
        self.position
    }

    pub(crate) fn valid(&self) -> bool {
        self.position.is_finite() && self.bearing.value.is_finite()
    }
}

impl From<InertialSensor> for InertialSensorTracking {
    fn from(sensor: InertialSensor) -> Self {
        Self::at_mount(
            sensor,
            (
                Vec3::new(Length::default(), Length::default(), Length::default()),
                Angle::default(),
            ),
        )
    }
}

impl InertialSensorTracking {
    pub(crate) async fn initialize(mut self) -> Option<Self> {
        self.calibrate().await?;
        Some(self)
    }

    async fn calibrate(&mut self) -> Option<()> {
        if !self.valid() {
            return None;
        }
        self.previous_rotation = None;
        let imu = &mut self.sensor;
        log_event!(
            Info,
            "IMU",
            "Calibrating sensor on port {}",
            imu.port_number()
        );
        if let Err(error) = imu.calibrate().await {
            log_event!(
                Error,
                "IMU",
                "Calibration failed on port {} ({error})",
                imu.port_number()
            );
            return None;
        }
        if let Err(error) = imu.reset_heading() {
            log_event!(
                Error,
                "IMU",
                "Could not zero heading on port {} ({error})",
                imu.port_number()
            );
            return None;
        }
        log_event!(Info, "IMU", "Sensor on port {} ready", imu.port_number());
        Some(())
    }

    pub(crate) fn read_orientation(&self, include_tilt: bool) -> Option<Orientation> {
        let imu = &self.sensor;
        if imu.is_calibrating().ok()? {
            return None;
        }
        let angle = |angle: vexide::math::Angle| {
            let radians = angle.as_radians();
            radians.is_finite().then(|| ozton_math::qty!((radians) rad))
        };
        let euler = include_tilt.then(|| imu.euler().ok()).flatten();
        let tilt = euler.and_then(|value| {
            let pitch = angle(value.a)?;
            let roll = angle(value.c.wrapped_half())?;
            Some(chassis_tilt(pitch, roll, self.bearing))
        });
        // The hardware reports clockwise headings and was zeroed at calibration.
        // Relocating the sensor or turning it about vertical does not change relative yaw.
        let sample = Orientation {
            port: imu.port_number(),
            heading: imu.heading().ok().and_then(angle),
            pitch: tilt.map(|(pitch, _)| pitch),
            roll: tilt.map(|(_, roll)| roll),
            yaw: euler.and_then(|value| angle(value.b)),
        };
        (sample.heading.is_some() || euler.is_some()).then_some(sample)
    }

    pub(crate) fn telemetry(&self) -> ([f64; 3], [f64; 3]) {
        let imu = &self.sensor;
        let rate = imu
            .gyro_rate()
            .map(|v| [v.x, v.y, v.z])
            .unwrap_or([f64::NAN; 3]);
        let accel = imu
            .acceleration()
            .map(|v| [v.x, v.y, v.z])
            .unwrap_or([f64::NAN; 3]);
        (rate, accel)
    }
}

impl ReadHeading for InertialSensorTracking {
    fn read_heading(&self) -> Option<Angle> {
        self.read_orientation(false)?.heading.map(compass_heading)
    }
}

impl ReadTilt for InertialSensorTracking {
    fn read_tilt(&self) -> Option<[Angle; 2]> {
        if self.sensor.is_calibrating().ok()? {
            return None;
        }
        let euler = self.sensor.euler().ok()?;
        let pitch = euler.a.as_radians();
        let roll = euler.c.wrapped_half().as_radians();
        if !pitch.is_finite() || !roll.is_finite() {
            return None;
        }
        let (pitch, roll) = chassis_tilt(
            ozton_math::qty!((pitch) rad),
            ozton_math::qty!((roll) rad),
            self.bearing,
        );
        Some([pitch, roll])
    }
}

impl crate::TrackingSource for InertialSensorTracking {
    type Capabilities = crate::capabilities::Orientation;

    fn initialize(&mut self) -> crate::SourceInitialization<'_> {
        Some(Box::pin(async {
            self.calibrate()
                .await
                .ok_or(crate::sampling::TrackingBuildError::ImuStartup)
        }))
    }

    fn sample(&mut self, sink: &mut crate::MeasurementSink<'_>) {
        use crate::{HeadingObservation, Measurement};

        self.last = self.read_orientation(true);
        if let Some(attitude) = self.last {
            if let Some(heading) = attitude.heading {
                sink.push(Measurement::Heading(HeadingObservation::Compass(heading)));
            }
            sink.push(Measurement::Attitude(attitude));
            let rotation = self.sensor.gyro_rate().ok().and_then(|rate| {
                // VEX gyro Z is CCW for an upright sensor; heading is clockwise.
                let sensor_rate = DVec3::new(
                    rate.x.to_radians(),
                    rate.y.to_radians(),
                    rate.z.to_radians(),
                );
                let rate = rotate_mount(sensor_rate, self.bearing.value);
                rate.is_finite().then_some(rate)
            });
            if let Some(rate) = rotation {
                sink.push(Measurement::Rotation(DVec3::Z * rate.z));
            }
            if self.planar_acceleration {
                let arm = DVec3::new(
                    self.position.right.value,
                    self.position.forward.value,
                    self.position.up.value,
                );
                let angular = angular_sample(&mut self.previous_rotation, sink.time, rotation);
                if let (Ok(accel), Ok(tilt)) = (self.sensor.acceleration(), self.sensor.euler()) {
                    let planar = planar_acceleration(
                        DVec2::new(accel.x, accel.y),
                        tilt.a.as_radians(),
                        tilt.c.as_radians(),
                        self.bearing.value,
                    );
                    // At the origin no gyro derivative is needed. Elsewhere,
                    // never substitute zero angular acceleration for a missing sample.
                    let corrected = if arm == DVec3::ZERO {
                        Some(planar)
                    } else {
                        angular.map(|(rate, alpha)| origin_acceleration(planar, arm, rate, alpha))
                    };
                    if let Some(planar) = corrected.filter(|value| value.is_finite()) {
                        sink.push(Measurement::Acceleration(planar.extend(0.0)));
                    }
                }
            }
        } else {
            self.previous_rotation = None;
        }
    }

    fn inertial(&self) -> Option<&Self> {
        Some(self)
    }
}

impl InertialSensorTracking {
    pub(crate) fn diagnostic_orientation(&self) -> Option<Orientation> {
        self.last
            .filter(|sample| sample.roll.is_some() || sample.pitch.is_some())
            .or_else(|| self.read_orientation(true))
    }
}

// Rotate a sensor-frame vector into the chassis right/forward/up frame.
fn rotate_mount(vector: DVec3, bearing: f64) -> DVec3 {
    let (sin, cos) = bearing.sin_cos();
    DVec3::new(
        vector.x * cos - vector.y * sin,
        vector.x * sin + vector.y * cos,
        vector.z,
    )
}

// Differentiate consecutive gyro observations using the tracking frame clock.
// Ignore repeated/too-close timestamps; rebaseline after gaps or missing data.
fn angular_sample(
    previous: &mut Option<(Duration, DVec3)>,
    time: Duration,
    rate: Option<DVec3>,
) -> Option<(DVec3, DVec3)> {
    let Some(rate) = rate.filter(|rate| rate.is_finite()) else {
        *previous = None;
        return None;
    };
    let Some((before, old_rate)) = *previous else {
        *previous = Some((time, rate));
        return None;
    };
    let Some(dt) = time.checked_sub(before) else {
        *previous = Some((time, rate));
        return None;
    };
    if dt < Duration::from_millis(5) {
        return None;
    }
    *previous = Some((time, rate));
    if dt > Duration::from_millis(100) {
        return None;
    }
    Some((rate, (rate - old_rate) / dt.as_secs_f64()))
}

// Rigid-body transport: a_sensor = a_origin + alpha × r + omega × (omega × r).
// All vectors use chassis coordinates in SI units; the estimator consumes XY.
fn origin_acceleration(planar: DVec2, arm: DVec3, rate: DVec3, alpha: DVec3) -> DVec2 {
    planar - (alpha.cross(arm) + rate.cross(rate.cross(arm))).truncate()
}

// SDK attitude convention observed on V5: a=pitch, c=roll. The
// accelerometer reports specific force including the static gravity projection.
// Sensor X points right and Y forward for the zero-bearing flat mount.
fn planar_acceleration(raw_g: glam::DVec2, pitch: f64, roll: f64, bearing: f64) -> glam::DVec2 {
    const STANDARD_GRAVITY: f64 = 9.80665;
    let gravity = glam::DVec2::new(pitch.sin(), roll.sin() * pitch.cos());
    let a = (raw_g - gravity) * STANDARD_GRAVITY;
    let (sin, cos) = bearing.sin_cos();
    glam::DVec2::new(a.x * cos - a.y * sin, a.x * sin + a.y * cos)
}

// Express gravity in the chassis's forward/right/down frame, then recover tilt.
fn chassis_tilt(pitch: Angle, roll: Angle, bearing: Angle) -> (Angle, Angle) {
    if bearing.value == 0.0 {
        return (pitch, roll);
    }
    let (sin_pitch, cos_pitch) = pitch.value.sin_cos();
    let (sin_roll, cos_roll) = roll.value.sin_cos();
    let (sin_bearing, cos_bearing) = bearing.value.sin_cos();
    let forward = -sin_pitch;
    let right = sin_roll * cos_pitch;
    let down = cos_roll * cos_pitch;
    let robot_forward = forward * cos_bearing + right * sin_bearing;
    let robot_right = right * cos_bearing - forward * sin_bearing;
    (
        ozton_math::qty!(((-robot_forward).atan2(robot_right.hypot(down))) rad),
        ozton_math::qty!((robot_right.atan2(down)) rad),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn planar_acceleration_removes_gravity_and_rotates_mount() {
        let (pitch, roll) = (0.1_f64, -0.2_f64);
        let static_g = glam::DVec2::new(pitch.sin(), roll.sin() * pitch.cos());
        assert!(planar_acceleration(static_g, pitch, roll, 0.0).length() < 1e-12);
        let forward = static_g + glam::DVec2::Y / 9.80665;
        let rotated = planar_acceleration(forward, pitch, roll, std::f64::consts::FRAC_PI_2);
        assert!(rotated.distance(-glam::DVec2::X) < 1e-12);
    }

    #[test]
    fn surveyed_mount_cancels_yaw_and_pitch_roll_acceleration() {
        // Override's measured IMU position, in meters.
        let arm = DVec3::new(-0.245161, 0.493578, 2.941929) * 0.0254;
        for (rate, alpha) in [
            (DVec3::Z * 3.0, DVec3::ZERO),
            (DVec3::ZERO, DVec3::Z * -4.0),
            (DVec3::new(0.3, -0.4, 2.0), DVec3::new(1.0, 2.0, -3.0)),
        ] {
            let actual = DVec2::new(0.7, -0.2);
            let measured = actual + (alpha.cross(arm) + rate.cross(rate.cross(arm))).truncate();
            assert!(origin_acceleration(measured, arm, rate, alpha).distance(actual) < 1e-12);
        }
        let yaw = DVec3::Z * 3.0;
        assert!(origin_acceleration(-arm.truncate() * 9.0, arm, yaw, DVec3::ZERO).length() < 1e-12);
        let pitch_alpha = DVec3::X * 2.0;
        assert!(
            origin_acceleration(DVec2::new(0.0, -2.0 * arm.z), arm, DVec3::ZERO, pitch_alpha)
                .length()
                < 1e-12
        );
    }

    #[test]
    fn angular_derivative_uses_fresh_tracking_timestamps() {
        let mut previous = None;
        let mut sample = |ms, rate| angular_sample(&mut previous, Duration::from_millis(ms), rate);
        assert!(sample(0, Some(DVec3::ZERO)).is_none());
        assert!(sample(0, Some(DVec3::X)).is_none());
        let (rate, alpha) = sample(10, Some(DVec3::Z * 0.02)).unwrap();
        assert_eq!(rate, DVec3::Z * 0.02);
        assert!(alpha.distance(DVec3::Z * 2.0) < 1e-12);
        assert!(sample(200, Some(DVec3::ZERO)).is_none());
        assert!(sample(210, None).is_none());
        assert!(sample(220, Some(DVec3::ZERO)).is_none());
        assert!(sample(230, Some(DVec3::ZERO)).is_some());
        assert!(sample(100, Some(DVec3::ZERO)).is_none());
    }

    #[test]
    fn rotated_mount_uses_chassis_angular_axes() {
        let bearing = std::f64::consts::FRAC_PI_2;
        assert!(rotate_mount(DVec3::X, bearing).distance(DVec3::Y) < 1e-12);
        assert!(rotate_mount(DVec3::Z, bearing).distance(DVec3::Z) < 1e-12);
    }

    #[test]
    fn quarter_turn_mount_swaps_sensor_pitch_into_chassis_roll() {
        let pitch = ozton_math::qty!(0.2 rad);
        let (chassis_pitch, chassis_roll) = chassis_tilt(
            pitch,
            Angle::default(),
            ozton_math::qty!((std::f64::consts::FRAC_PI_2) rad),
        );
        assert!(chassis_pitch.value.abs() < 1e-12);
        assert!((chassis_roll.value - pitch.value).abs() < 1e-12);
    }
}
