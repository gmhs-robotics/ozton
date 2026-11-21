//! Shared calibrated IMU; initialization never waits in the Driver loop.
use std::{cell::OnceCell, rc::Rc, time::Duration};

use ozton_math::uom::si::{angle::radian, f64::Angle};
use vexide::{prelude::InertialSensor, smart::SmartDevice, task::Task, time::sleep};

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

/// Calibrates the sensor in the background and shares it when ready.
pub(super) struct CalibratedImu {
    ready: Rc<OnceCell<Rc<InertialSensor>>>,
    _task: Task<()>,
}

impl CalibratedImu {
    pub(super) fn start(mut imu: InertialSensor) -> Self {
        let ready = Rc::new(OnceCell::new());
        let calibrated = ready.clone();
        let task = vexide::task::spawn(async move {
            loop {
                if imu.calibrate().await.is_ok() && imu.reset_heading().is_ok() {
                    let _ = calibrated.set(Rc::new(imu));

                    break;
                }

                sleep(Duration::from_secs(1)).await;
            }
        });
        Self { ready, _task: task }
    }

    pub(super) fn sample(&self) -> Option<Orientation> {
        let imu = self.ready.get()?;
        if imu.is_calibrating().ok()? {
            return None;
        }
        let angle = |angle: vexide::math::Angle| {
            let radians = angle.as_radians();
            radians.is_finite().then(|| Angle::new::<radian>(radians))
        };
        let euler = imu.euler().ok();
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
        let Some(imu) = self.ready.get() else {
            return ([f64::NAN; 3], [f64::NAN; 3]);
        };
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
