//! Compatibility for a distance status rejected by vexide.

use vex_sdk::{vexDeviceDistanceConfidenceGet, vexDeviceDistanceDistanceGet, vexDeviceGetByIndex};
use vexide::{
    prelude::DistanceSensor,
    smart::{SmartDevice, distance::DistanceObjectError},
};

pub(super) const ALTERNATE_READY_STATUS: u32 = 0x84;

pub(super) fn read_object(sensor: &DistanceSensor) -> Result<Option<(u32, f64)>, ()> {
    match sensor.object() {
        Ok(reading) => Ok(reading.map(|object| (object.distance, object.confidence))),
        Err(DistanceObjectError::BadStatusCode {
            code: ALTERNATE_READY_STATUS,
        }) => {
            let index = u32::from(sensor.port_number() - 1);
            // SAFETY: The owned SmartPort supplies a valid zero-based SDK index.
            let handle = unsafe { vexDeviceGetByIndex(index) };
            if handle.is_null() {
                return Err(());
            }
            // SAFETY: object() validated the device type before reporting the
            // status error. The handle refers to that same VEXos-managed port.
            let (distance, confidence) = unsafe {
                (
                    vexDeviceDistanceDistanceGet(handle),
                    vexDeviceDistanceConfidenceGet(handle),
                )
            };
            // As in vexide, 9999 means no object and confidence is scaled by 63.
            Ok((distance != 9999).then_some((distance, f64::from(confidence) / 63.0)))
        }
        Err(_) => Err(()),
    }
}
