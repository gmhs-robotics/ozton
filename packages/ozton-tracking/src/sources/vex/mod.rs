//! Vexide adapters. Call capture once per control tick.

use std::time::Duration;

use ozton_math::{geometry::compass_heading, uom::si::f64::Length};
use vexide::{prelude::InertialSensor, time::LowResolutionTime};

use super::{
    Snapshot,
    distance::{MAX_RANGES, RangeObservation},
};
use crate::{RotarySensor, wheeled::TrackingWheel};

mod distance;
mod imu;
pub use distance::{DistanceTracker, Distances};
use imu::CalibratedImu;
pub use imu::Orientation;

/// A wheel-travel source in meters. A missing observation returns None.
pub trait WheelSource {
    fn read_travel(&self) -> Option<Length>;

    /// Performs any configuration checks before the source is installed.
    fn valid(&self) -> bool {
        true
    }
}

impl WheelSource for () {
    fn read_travel(&self) -> Option<Length> {
        None
    }
}

impl<T: RotarySensor> WheelSource for TrackingWheel<T> {
    fn read_travel(&self) -> Option<Length> {
        self.travel().ok()
    }

    fn valid(&self) -> bool {
        calibrated(self)
    }
}

type TravelCallback = Option<Box<dyn Fn() -> Option<Length>>>;

impl WheelSource for TravelCallback {
    fn read_travel(&self) -> Option<Length> {
        self.as_ref().and_then(|read| read())
    }
}

/// VEX sensor ownership and polling. Use [`Self::with_wheel_sources`] for
/// concrete wheel sources held inline and read through static dispatch.
pub struct Sensors<F = TravelCallback, L = TravelCallback, D = Distances> {
    imu: Option<CalibratedImu>,
    orientation: Option<Orientation>,
    forward: F,
    lateral: L,
    distances: D,
}

/// Wheel and heading reader paired with a separately owned distance bank.
pub type SharedSensors<F = TravelCallback, L = TravelCallback> = Sensors<F, L, ()>;

impl Default for Sensors {
    fn default() -> Self {
        Self::from_sources(None, None)
    }
}

impl<F: WheelSource, L: WheelSource> Sensors<F, L> {
    fn from_sources(forward: F, lateral: L) -> Self {
        Self {
            imu: None,
            orientation: None,
            forward,
            lateral,
            distances: Distances::default(),
        }
    }

    /// Starts with concrete wheel sources. Pass `()` for an unused axis.
    pub fn with_wheel_sources(forward: F, lateral: L) -> Option<Self> {
        (forward.valid() && lateral.valid()).then(|| Self::from_sources(forward, lateral))
    }

    /// Give this reader its own distance devices when no other component uses them.
    pub async fn with_ranges(
        mut self,
        ranges: impl IntoIterator<Item = DistanceTracker>,
    ) -> Option<Self> {
        self.distances = Distances::new(ranges)?.initialize().await;
        Some(self)
    }

    pub fn ready_range_count(&self) -> usize {
        self.distances.ready_count()
    }

    /// Sample the locally owned distance devices along with wheel and IMU data.
    pub fn capture(
        &mut self,
        drive: [Option<Length>; 2],
        occluded: u8,
    ) -> (Snapshot, [Option<RangeObservation>; MAX_RANGES]) {
        self.refresh_orientation();
        let ranges = self.distances.capture(occluded);
        (self.capture_odometry(drive), ranges)
    }

    /// Prepare this reader for distances owned on the parent tracking structure.
    /// Existing distance devices must be managed by the caller before conversion.
    pub fn into_shared_reader(self) -> SharedSensors<F, L> {
        let Self {
            imu,
            orientation,
            forward,
            lateral,
            ..
        } = self;
        Sensors {
            imu,
            orientation,
            forward,
            lateral,
            distances: (),
        }
    }
}

impl<F: WheelSource, L: WheelSource, D> Sensors<F, L, D> {
    /// Calibrate once during assembly. A failed calibration returns no reader.
    pub async fn with_imu(mut self, imu: InertialSensor) -> Option<Self> {
        self.imu = Some(CalibratedImu::initialize(imu).await?);
        self.orientation = None;
        Some(self)
    }

    pub fn refresh_orientation(&mut self) -> Option<Orientation> {
        self.sample_orientation(true)
    }

    /// Refresh the cached heading alone when tilt is unused.
    pub fn refresh_heading(&mut self) -> Option<Orientation> {
        self.sample_orientation(false)
    }

    fn sample_orientation(&mut self, include_tilt: bool) -> Option<Orientation> {
        self.orientation = self.imu.as_ref().and_then(|imu| imu.sample(include_tilt));
        self.orientation
    }

    pub fn orientation(&self) -> Option<Orientation> {
        self.orientation
    }

    /// Full orientation for a diagnostics sink, independent of the odometry inputs.
    pub fn diagnostic_orientation(&self) -> Option<Orientation> {
        self.orientation
            .filter(|sample| sample.roll.is_some() || sample.pitch.is_some())
            .or_else(|| self.imu.as_ref()?.sample(true))
    }

    pub fn imu_telemetry(&self) -> ([f64; 3], [f64; 3]) {
        self.imu
            .as_ref()
            .map_or(([f64::NAN; 3], [f64::NAN; 3]), CalibratedImu::telemetry)
    }

    /// No range device calls occur on this path.
    pub fn capture_odometry(&self, drive: [Option<Length>; 2]) -> Snapshot {
        let imu = self
            .orientation
            .and_then(|sample| sample.heading)
            .map(compass_heading);
        let forward = self.forward.read_travel();
        let lateral = self.lateral.read_travel();
        Snapshot {
            time: stamp(LowResolutionTime::now()),
            drive,
            forward,
            lateral,
            imu,
        }
    }
}

impl<F: WheelSource, L: WheelSource> SharedSensors<F, L> {
    /// Sample shared distance devices and the other sensors in a single tick.
    pub fn capture_with_distances(
        &self,
        distances: &mut Distances,
        drive: [Option<Length>; 2],
        occluded: u8,
    ) -> (Snapshot, [Option<RangeObservation>; MAX_RANGES]) {
        let ranges = distances.capture(occluded);
        (self.capture_odometry(drive), ranges)
    }
}

impl Sensors {
    /// Pass calibrated wheel travel in meters, or None when no valid reading is available.
    pub fn with_forward(mut self, read: impl Fn() -> Option<Length> + 'static) -> Self {
        self.forward = Some(Box::new(read));
        self
    }

    pub fn with_lateral(mut self, read: impl Fn() -> Option<Length> + 'static) -> Self {
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
}

fn calibrated<T: RotarySensor>(wheel: &TrackingWheel<T>) -> bool {
    wheel.wheel_diameter.value.is_finite()
        && wheel.wheel_diameter.value > 0.0
        && wheel
            .gearing
            .is_none_or(|ratio| ratio.is_finite() && ratio > 0.0)
}

fn stamp(time: LowResolutionTime) -> Duration {
    time.duration_since(LowResolutionTime::EPOCH)
}

#[cfg(test)]
mod tests {
    use ozton_math::uom::si::length::meter;
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
        let wheel = TrackingWheel::new(
            Encoder(-2.0),
            Length::new::<meter>(0.1),
            Length::new::<meter>(0.0),
            Some(0.5),
        );
        let sensors = Sensors::with_wheel_sources((), wheel).unwrap();
        let sample = sensors.capture_odometry([Some(Length::new::<meter>(0.1)), None]);
        assert_eq!(sample.drive, [Some(Length::new::<meter>(0.1)), None]);
        assert_eq!(sample.forward, None);
        assert!((sample.lateral.unwrap().value + 0.1 * core::f64::consts::PI).abs() < 1e-12);

        for diameter in [0.0, -0.1, f64::NAN] {
            let wheel = TrackingWheel::new(
                Encoder(1.0),
                Length::new::<meter>(diameter),
                Length::new::<meter>(0.0),
                None,
            );
            assert!(Sensors::with_wheel_sources(wheel, ()).is_none());
        }
    }
}
