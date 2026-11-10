//! Vexide adapters. Call capture once per control tick.

use std::{rc::Rc, time::Duration};

use ozton_math::geometry::{Pose, compass_heading};
use vexide::{
    prelude::{DistanceSensor, GpsSensor, InertialSensor},
    smart::SmartDevice,
    time::LowResolutionTime,
};

use super::{GpsFix, MAX_RANGES, RangeObservation, Snapshot};
use crate::{RotarySensor, wheeled::TrackingWheel};

const RANGE_PERIOD: Duration = Duration::from_millis(33);

/// Sensor ownership for a typical differential chassis.
#[derive(Default)]
pub struct Sensors {
    pub imu: Option<Rc<InertialSensor>>,
    pub gps: Option<GpsSensor>,
    forward: Option<Box<dyn Fn() -> Option<f64>>>,
    lateral: Option<Box<dyn Fn() -> Option<f64>>>,
    ranges: Vec<DistanceSensor>,
    last_range_poll: Option<Duration>,
    cached_ranges: [Option<RangeObservation>; MAX_RANGES],
}

impl Sensors {
    /// Pass calibrated wheel travel in meters, or None when no valid reading is available.
    pub fn with_forward(mut self, read: impl Fn() -> Option<f64> + 'static) -> Self {
        self.forward = Some(Box::new(read));
        self
    }

    pub fn with_lateral(mut self, read: impl Fn() -> Option<f64> + 'static) -> Self {
        self.lateral = Some(Box::new(read));
        self
    }

    pub fn with_forward_wheel<T: RotarySensor + 'static>(
        self,
        wheel: TrackingWheel<T>,
    ) -> Option<Self> {
        calibrated(&wheel).then(|| self.with_forward(move || wheel.travel().ok()))
    }

    pub fn with_lateral_wheel<T: RotarySensor + 'static>(
        self,
        wheel: TrackingWheel<T>,
    ) -> Option<Self> {
        calibrated(&wheel).then(|| self.with_lateral(move || wheel.travel().ok()))
    }

    /// Order must match the localizer's range configurations.
    pub fn with_ranges(mut self, ranges: impl IntoIterator<Item = DistanceSensor>) -> Option<Self> {
        self.ranges = ranges.into_iter().take(MAX_RANGES + 1).collect();
        self.last_range_poll = None;
        self.cached_ranges.fill(None);
        (self.ranges.len() <= MAX_RANGES).then_some(self)
    }

    /// Sample motor positions separately through the drivetrain. Mask sensors obscured
    /// by the robot's mechanisms or a known game object.
    pub fn capture(
        &mut self,
        drive: [Option<f64>; 2],
        occluded: u8,
    ) -> (Snapshot, [Option<RangeObservation>; MAX_RANGES]) {
        let time = stamp(LowResolutionTime::now());
        if self
            .last_range_poll
            .is_none_or(|last| time.saturating_sub(last) >= RANGE_PERIOD)
        {
            for (index, sensor) in self.ranges.iter().enumerate() {
                self.cached_ranges[index] = distance(sensor, occluded & (1 << index) != 0);
            }
            self.last_range_poll = Some(time);
        }
        let mut ranges = self.cached_ranges;
        for (index, sample) in ranges.iter_mut().enumerate() {
            if let Some(sample) = sample {
                sample.occluded |= occluded & (1 << index) != 0;
            }
        }
        let imu = self.imu.as_deref().and_then(|imu| {
            if imu.is_calibrating().ok()? {
                return None;
            }
            Some(compass_heading(imu.heading().ok()?.as_degrees()))
        });
        let gps = self.gps.as_ref().and_then(gps_fix);
        let forward = self.forward.as_ref().and_then(|read| read());
        let lateral = self.lateral.as_ref().and_then(|read| read());
        (
            Snapshot {
                time: stamp(LowResolutionTime::now()),
                drive,
                forward,
                lateral,
                imu,
                gps,
            },
            ranges,
        )
    }
}

fn calibrated<T: RotarySensor>(wheel: &TrackingWheel<T>) -> bool {
    wheel.wheel_diameter.is_finite()
        && wheel.wheel_diameter > 0.0
        && wheel
            .gearing
            .is_none_or(|ratio| ratio.is_finite() && ratio > 0.0)
}

fn stamp(time: LowResolutionTime) -> Duration {
    time.duration_since(LowResolutionTime::EPOCH)
}

fn distance(sensor: &DistanceSensor, occluded: bool) -> Option<RangeObservation> {
    let before = sensor.timestamp().ok()?;
    let reading = sensor.object().ok()??;
    let after = sensor.timestamp().ok()?;
    if before != after {
        return None;
    }
    Some(RangeObservation {
        timestamp: stamp(after),
        distance: f64::from(reading.distance) * 0.001,
        confidence: reading.confidence,
        occluded,
    })
}

fn gps_fix(sensor: &GpsSensor) -> Option<GpsFix> {
    let before = sensor.timestamp().ok()?;
    let position = sensor.position().ok()?;
    let heading = compass_heading(sensor.heading().ok()?.as_degrees());
    let position_error = sensor.error().ok()?;
    let after = sensor.timestamp().ok()?;
    if before != after {
        return None;
    }
    Some(GpsFix {
        pose: Pose::new(position.x, position.y, heading),
        position_error,
        timestamp: stamp(after),
    })
}
