//! IMU calibrated once before competition.

use ozton_math::uom::si::{angle::radian, f64::Angle};
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

pub(super) struct CalibratedImu(InertialSensor);

impl CalibratedImu {
    pub(super) async fn initialize(mut imu: InertialSensor) -> Option<Self> {
        imu.calibrate().await.ok()?;
        imu.reset_heading().ok()?;
        Some(Self(imu))
    }

    pub(super) fn sample(&self, include_tilt: bool) -> Option<Orientation> {
        let imu = &self.0;
        if imu.is_calibrating().ok()? {
            return None;
        }
        let angle = |angle: vexide::math::Angle| {
            let radians = angle.as_radians();
            radians.is_finite().then(|| Angle::new::<radian>(radians))
        };
        let euler = include_tilt.then(|| imu.euler().ok()).flatten();
        let sample = Orientation {
            port: imu.port_number(),
            heading: imu.heading().ok().and_then(angle),
            pitch: euler.and_then(|value| angle(value.a)),
            roll: euler.and_then(|value| angle(value.c.wrapped_half())),
            yaw: euler.and_then(|value| angle(value.b)),
        };
        (sample.heading.is_some() || euler.is_some()).then_some(sample)
    }

    pub(super) fn telemetry(&self) -> ([f64; 3], [f64; 3]) {
        let imu = &self.0;
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
