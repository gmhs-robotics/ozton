//! Shared calibrated IMU; initialization never waits in the Driver loop.
use std::{cell::OnceCell, rc::Rc, time::Duration};

use vexide::{math::Angle, prelude::InertialSensor, task::Task, time::sleep};

/// Calibrates the sensor in the background and shares it when ready.
pub struct CalibratedImu {
    ready: Rc<OnceCell<Rc<InertialSensor>>>,
    _task: Task<()>,
}

impl CalibratedImu {
    pub fn start(mut imu: InertialSensor) -> Self {
        let ready = Rc::new(OnceCell::new());
        let calibrated = ready.clone();
        let task = vexide::task::spawn(async move {
            loop {
                if imu.calibrate().await.is_ok() && imu.reset_heading().is_ok() {
                    let _ = calibrated.set(Rc::new(imu));
                    crate::diagnostic_event!("imu_ready,heading_assist=1");
                    break;
                }
                crate::diagnostic_event!("imu_unavailable,manual_drive_available=1");
                sleep(Duration::from_secs(1)).await;
            }
        });
        Self { ready, _task: task }
    }

    pub fn sensor(&self) -> Option<Rc<InertialSensor>> {
        self.ready.get().cloned()
    }

    pub fn heading(&self) -> Option<Angle> {
        let sensor = self.sensor()?;
        if sensor.is_calibrating().ok()? {
            return None;
        }
        sensor
            .heading()
            .ok()
            .filter(|angle| angle.as_radians().is_finite())
    }
}
