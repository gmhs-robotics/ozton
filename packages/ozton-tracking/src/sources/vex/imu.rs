//! IMU calibrated once before competition.

use ozton_diagnostics::log_event;
use ozton_math::uom::si::{
    angle::radian,
    f64::{Angle, Length},
};
use vexide::{prelude::InertialSensor, smart::SmartDevice};

/// Orientation read once for a control tick. Heading follows the IMU's
/// clockwise convention, while roll and pitch follow VEX body axes.
#[derive(Debug, Clone, Copy)]
pub struct Orientation {
    pub port: u8,
    pub heading: Option<Angle>,
    pub pitch: Option<Angle>,
    pub roll: Option<Angle>,
    pub yaw: Option<Angle>,
}

use crate::mount::Vec3;

/// An IMU mounted flat, with a counterclockwise bearing from chassis forward.
/// Position is kept for surveying. Planar heading is independent of the IMU offset.
pub struct InertialSensorTracking {
    sensor: InertialSensor,
    bearing: Angle,
    position: Vec3<Length>,
}

impl InertialSensorTracking {
    pub fn new(sensor: InertialSensor, bearing: Angle, position: Vec3<Length>) -> Self {
        Self {
            sensor,
            bearing,
            position,
        }
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
        Self::new(
            sensor,
            Angle::default(),
            Vec3::new(Length::default(), Length::default(), Length::default()),
        )
    }
}

pub(super) struct CalibratedImu(InertialSensorTracking);

impl CalibratedImu {
    pub(super) async fn initialize(mut mounted: InertialSensorTracking) -> Option<Self> {
        if !mounted.valid() {
            return None;
        }
        let imu = &mut mounted.sensor;
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
        Some(Self(mounted))
    }

    pub(super) fn sample(&self, include_tilt: bool) -> Option<Orientation> {
        let imu = &self.0.sensor;
        if imu.is_calibrating().ok()? {
            return None;
        }
        let angle = |angle: vexide::math::Angle| {
            let radians = angle.as_radians();
            radians.is_finite().then(|| Angle::new::<radian>(radians))
        };
        let euler = include_tilt.then(|| imu.euler().ok()).flatten();
        let tilt = euler.and_then(|value| {
            let pitch = angle(value.a)?;
            let roll = angle(value.c.wrapped_half())?;
            Some(chassis_tilt(pitch, roll, self.0.bearing))
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

    pub(super) fn telemetry(&self) -> ([f64; 3], [f64; 3]) {
        let imu = &self.0.sensor;
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
        Angle::new::<radian>((-robot_forward).atan2(robot_right.hypot(down))),
        Angle::new::<radian>(robot_right.atan2(down)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quarter_turn_mount_swaps_sensor_pitch_into_chassis_roll() {
        let pitch = Angle::new::<radian>(0.2);
        let (chassis_pitch, chassis_roll) = chassis_tilt(
            pitch,
            Angle::default(),
            Angle::new::<radian>(std::f64::consts::FRAC_PI_2),
        );
        assert!(chassis_pitch.value.abs() < 1e-12);
        assert!((chassis_roll.value - pitch.value).abs() < 1e-12);
    }
}
