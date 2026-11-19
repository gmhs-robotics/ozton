//! Vexide adapters. Call capture once per control tick.

use std::{rc::Rc, time::Duration};

use arrayvec::ArrayVec;
use ozton_math::{
    geometry::{Pose, compass_heading},
    units::millimeters,
};
use vexide::{
    prelude::{DistanceSensor, GpsSensor, InertialSensor},
    smart::SmartDevice,
    time::LowResolutionTime,
};

use super::{GpsFix, MAX_RANGES, RangeObservation, Snapshot};
use crate::{RotarySensor, wheeled::TrackingWheel};

const RANGE_PERIOD: Duration = Duration::from_millis(33);

/// A wheel-travel source in meters. A missing observation returns None.
pub trait WheelSource {
    fn read_travel(&self) -> Option<f64>;

    /// Performs any configuration checks before the source is installed.
    fn valid(&self) -> bool {
        true
    }
}

impl WheelSource for () {
    fn read_travel(&self) -> Option<f64> {
        None
    }
}

impl<T: RotarySensor> WheelSource for TrackingWheel<T> {
    fn read_travel(&self) -> Option<f64> {
        self.travel().ok()
    }

    fn valid(&self) -> bool {
        calibrated(self)
    }
}

/// Optional, type-erased source used by the callback builders.
#[derive(Default)]
pub struct CallbackWheel(Option<Box<dyn Fn() -> Option<f64>>>);

impl WheelSource for CallbackWheel {
    fn read_travel(&self) -> Option<f64> {
        self.0.as_ref().and_then(|read| read())
    }
}

/// VEX sensor ownership and polling. Use [`Self::with_wheel_sources`] for
/// concrete wheel sources held inline and read through static dispatch.
pub struct Sensors<F = CallbackWheel, L = CallbackWheel> {
    pub imu: Option<Rc<InertialSensor>>,
    pub gps: Option<GpsSensor>,
    field_origin: Pose,
    forward: F,
    lateral: L,
    ranges: ArrayVec<DistanceSensor, MAX_RANGES>,
    last_range_poll: Option<Duration>,
    cached_ranges: [Option<RangeObservation>; MAX_RANGES],
}

impl Default for Sensors {
    fn default() -> Self {
        Self::from_sources(CallbackWheel::default(), CallbackWheel::default())
    }
}

impl<F: WheelSource, L: WheelSource> Sensors<F, L> {
    fn from_sources(forward: F, lateral: L) -> Self {
        Self {
            imu: None,
            gps: None,
            field_origin: Pose::default(),
            forward,
            lateral,
            ranges: ArrayVec::new(),
            last_range_poll: None,
            cached_ranges: [None; MAX_RANGES],
        }
    }

    /// Starts with concrete wheel sources. Pass `()` for an unused axis.
    pub fn with_wheel_sources(forward: F, lateral: L) -> Option<Self> {
        (forward.valid() && lateral.valid()).then(|| Self::from_sources(forward, lateral))
    }

    /// Align centered GPS readings with the same surveyed route origin used by
    /// walls, targets and the localizer. Default is the centered GPS field frame.
    pub fn set_field_origin(&mut self, origin: Pose) -> bool {
        if !origin.is_finite() {
            return false;
        }
        self.field_origin = origin;
        true
    }

    /// Order must match the localizer's range configurations. Returns None above MAX_RANGES.
    pub fn with_ranges(mut self, ranges: impl IntoIterator<Item = DistanceSensor>) -> Option<Self> {
        self.ranges.clear();
        for sensor in ranges {
            self.ranges.try_push(sensor).ok()?;
        }
        self.last_range_poll = None;
        self.cached_ranges.fill(None);
        Some(self)
    }

    /// Count initialized, responsive range devices, including those that see no object.
    pub fn ready_range_count(&self) -> usize {
        self.ranges
            .iter()
            .filter(|sensor| sensor.object().is_ok())
            .count()
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
        let gps = self.gps.as_ref().and_then(gps_fix).map(|mut fix| {
            fix.pose = fix.pose.in_frame(self.field_origin);
            fix
        });
        let mut snapshot = self.capture_odometry(drive);
        snapshot.gps = gps;
        (snapshot, ranges)
    }

    /// Read only the travel and heading required for low-latency driver odometry.
    /// No range or GPS device calls occur on this path.
    pub fn capture_odometry(&self, drive: [Option<f64>; 2]) -> Snapshot {
        let imu = self.imu.as_deref().and_then(|imu| {
            if imu.is_calibrating().ok()? {
                return None;
            }
            Some(compass_heading(imu.heading().ok()?.as_degrees()))
        });
        let forward = self.forward.read_travel();
        let lateral = self.lateral.read_travel();
        Snapshot {
            time: stamp(LowResolutionTime::now()),
            drive,
            forward,
            lateral,
            imu,
            gps: None,
        }
    }
}

impl Sensors {
    /// Pass calibrated wheel travel in meters, or None when no valid reading is available.
    pub fn with_forward(mut self, read: impl Fn() -> Option<f64> + 'static) -> Self {
        self.forward.0 = Some(Box::new(read));
        self
    }

    pub fn with_lateral(mut self, read: impl Fn() -> Option<f64> + 'static) -> Self {
        self.lateral.0 = Some(Box::new(read));
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
        distance: millimeters(f64::from(reading.distance)).value,
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

#[cfg(test)]
mod tests {
    use vexide::math::Angle;

    use super::*;

    struct Encoder(f64);

    impl RotarySensor for Encoder {
        type Error = core::convert::Infallible;

        fn position(&self) -> Result<Angle, Self::Error> {
            Ok(Angle::from_turns(self.0))
        }
    }

    #[test]
    fn concrete_wheels_sample_each_axis_without_callbacks() {
        let wheel = TrackingWheel::new(Encoder(-2.0), 0.1, 0.0, Some(0.5));
        let sensors = Sensors::with_wheel_sources((), wheel).unwrap();
        let sample = sensors.capture_odometry([Some(0.1), None]);
        assert_eq!(sample.drive, [Some(0.1), None]);
        assert_eq!(sample.forward, None);
        assert!((sample.lateral.unwrap() + 0.1 * core::f64::consts::PI).abs() < 1e-12);

        for diameter in [0.0, -0.1, f64::NAN] {
            let wheel = TrackingWheel::new(Encoder(1.0), diameter, 0.0, None);
            assert!(Sensors::with_wheel_sources(wheel, ()).is_none());
        }
    }

    #[test]
    fn existing_callback_sources_can_still_be_replaced() {
        let sensors = Sensors::default()
            .with_forward(|| Some(1.0))
            .with_forward(|| Some(2.0))
            .with_lateral(|| Some(-3.0));
        let sample = sensors.capture_odometry([None; 2]);
        assert_eq!((sample.forward, sample.lateral), (Some(2.0), Some(-3.0)));
    }
}
