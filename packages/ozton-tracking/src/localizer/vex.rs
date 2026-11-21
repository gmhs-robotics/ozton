//! Vexide adapters. Call capture once per control tick.

use std::{
    cell::Cell,
    future::Future,
    pin::pin,
    rc::Rc,
    task::{Context, Poll, Waker},
    time::Duration,
};

use arrayvec::ArrayVec;
use ozton_math::{
    geometry::{Pose, compass_heading},
    uom::si::{
        angle::radian,
        f64::{Angle as UomAngle, Length},
        length::{meter, millimeter},
    },
};
use vexide::{
    prelude::{DistanceSensor, GpsSensor, InertialSensor},
    smart::{SmartDevice, SmartPort},
    task::Task,
    time::LowResolutionTime,
};

use super::{GpsFix, MAX_RANGES, RangeObservation, RangeSensorConfig, Snapshot};
use crate::{RotarySensor, mount::Vec3, wheeled::TrackingWheel};

mod imu;
use imu::CalibratedImu;
pub use imu::Orientation;

const RANGE_PERIOD: Duration = Duration::from_millis(33);

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

/// An owned distance sensor and its surveyed right/forward/up mounting point.
pub struct DistanceTracker {
    sensor: DistanceSensor,
    config: RangeSensorConfig,
}

impl DistanceTracker {
    /// Bearing is counterclockwise from forward.
    pub fn new(port: SmartPort, position: Vec3<Length>, bearing: UomAngle) -> Self {
        Self {
            sensor: DistanceSensor::new(port),
            config: RangeSensorConfig::at_mount(position, bearing),
        }
    }

    /// Change measurement tuning without replacing the surveyed mount.
    pub fn with_tuning(
        mut self,
        gate: ozton_math::localization::range::RangeGate,
        bias: Length,
        sigma: Length,
    ) -> Self {
        self.config.gate = gate;
        self.config.bias = bias;
        self.config.sigma = sigma;
        self
    }
}

#[derive(Clone, Copy, Default)]
struct RangeReadiness {
    ready: bool,
    generation: u32,
}

impl RangeReadiness {
    fn set(&mut self, ready: bool) {
        if self.ready && !ready {
            self.generation = self.generation.wrapping_add(1);
        }
        self.ready = ready;
    }

    fn current<T>(self, cached: Option<(u32, T)>) -> Option<T> {
        let (generation, value) = cached?;
        (self.ready && self.generation == generation).then_some(value)
    }
}

struct DistanceDevice {
    sensor: DistanceSensor,
    readiness: Cell<RangeReadiness>,
}

impl DistanceDevice {
    fn set_ready(&self, ready: bool) {
        let mut readiness = self.readiness.get();
        readiness.set(ready);
        self.readiness.set(readiness);
    }
}

/// Distance devices polled once per packet window and shared by higher-level
/// consumers through the cached observations.
#[derive(Default)]
pub struct Distances {
    devices: ArrayVec<Rc<DistanceDevice>, MAX_RANGES>,
    configs: ArrayVec<RangeSensorConfig, MAX_RANGES>,
    _readiness: Option<Task<()>>,
    last_poll: Option<Duration>,
    last_occluded: u8,
    cached: [Option<(u32, RangeObservation)>; MAX_RANGES],
}

impl Distances {
    /// Register mounts in route index order and start background readiness checks.
    pub fn new(devices: impl IntoIterator<Item = DistanceTracker>) -> Option<Self> {
        let mut distances = Self::default();
        for DistanceTracker { sensor, config } in devices {
            distances
                .devices
                .try_push(Rc::new(DistanceDevice {
                    sensor,
                    readiness: Cell::default(),
                }))
                .ok()?;
            distances.configs.push(config);
        }
        if !distances.is_empty() {
            let devices = distances.devices.clone();
            distances._readiness = Some(vexide::task::spawn(async move {
                loop {
                    for device in &devices {
                        // The SDK future wakes itself while initializing. Pace it
                        // here so an unready sensor cannot busy-loop the executor.
                        let mut ready = pin!(device.sensor.wait_ready());
                        let result = ready.as_mut().poll(&mut Context::from_waker(Waker::noop()));
                        device.set_ready(matches!(result, Poll::Ready(Ok(()))));
                    }
                    vexide::time::sleep(RANGE_PERIOD).await;
                }
            }));
        }
        Some(distances)
    }

    pub fn configs(&self) -> &[RangeSensorConfig] {
        &self.configs
    }

    pub fn len(&self) -> usize {
        self.devices.len()
    }

    pub fn is_empty(&self) -> bool {
        self.devices.is_empty()
    }

    /// Count responsive devices, including those that see no object.
    pub fn ready_count(&self) -> usize {
        self.devices
            .iter()
            .filter(|device| device.readiness.get().ready)
            .count()
    }

    /// Return the most recent packet without another device read.
    pub fn latest(&self, index: usize) -> Option<RangeObservation> {
        let readiness = self.devices.get(index)?.readiness.get();
        let mut observation = readiness.current(self.cached[index])?;
        observation.occluded |= self.last_occluded & (1 << index) != 0;
        Some(observation)
    }

    fn capture(&mut self, occluded: u8) -> [Option<RangeObservation>; MAX_RANGES] {
        let time = stamp(LowResolutionTime::now());
        if self
            .last_poll
            .is_none_or(|last| time.saturating_sub(last) >= RANGE_PERIOD)
        {
            for (index, device) in self.devices.iter().enumerate() {
                let readiness = device.readiness.get();
                self.cached[index] = if readiness.ready {
                    match distance(&device.sensor, occluded & (1 << index) != 0) {
                        Ok(observation) => observation.map(|value| (readiness.generation, value)),
                        Err(()) => {
                            device.set_ready(false);
                            None
                        }
                    }
                } else {
                    None
                };
            }
            self.last_poll = Some(time);
        }
        self.last_occluded = occluded;
        core::array::from_fn(|index| self.latest(index))
    }
}

/// VEX sensor ownership and polling. Use [`Self::with_wheel_sources`] for
/// concrete wheel sources held inline and read through static dispatch.
pub struct Sensors<F = TravelCallback, L = TravelCallback, D = Distances> {
    imu: Option<CalibratedImu>,
    orientation: Option<Orientation>,
    pub gps: Option<GpsSensor>,
    field_origin: Pose,
    forward: F,
    lateral: L,
    distances: D,
}

/// Wheel, heading and GPS reader paired with a separately owned distance bank.
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
            gps: None,
            field_origin: Pose::default(),
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
    pub fn with_ranges(
        mut self,
        ranges: impl IntoIterator<Item = DistanceTracker>,
    ) -> Option<Self> {
        self.distances = Distances::new(ranges)?;
        Some(self)
    }

    pub fn ready_range_count(&self) -> usize {
        self.distances.ready_count()
    }

    /// Sample the locally owned distance devices along with wheel, IMU and GPS data.
    pub fn capture(
        &mut self,
        drive: [Option<Length>; 2],
        occluded: u8,
    ) -> (Snapshot, [Option<RangeObservation>; MAX_RANGES]) {
        self.refresh_orientation();
        let ranges = self.distances.capture(occluded);
        (self.capture_snapshot(drive), ranges)
    }

    /// Prepare this reader for distances owned on the parent tracking structure.
    /// Existing distance devices must be managed by the caller before conversion.
    pub fn into_shared_reader(self) -> SharedSensors<F, L> {
        let Self {
            imu,
            orientation,
            gps,
            field_origin,
            forward,
            lateral,
            ..
        } = self;
        Sensors {
            imu,
            orientation,
            gps,
            field_origin,
            forward,
            lateral,
            distances: (),
        }
    }
}

impl<F: WheelSource, L: WheelSource, D> Sensors<F, L, D> {
    /// Start background calibration of the tracking IMU.
    pub fn with_imu(mut self, imu: InertialSensor) -> Self {
        self.imu = Some(CalibratedImu::start(imu));
        self.orientation = None;
        self
    }

    pub fn refresh_orientation(&mut self) -> Option<Orientation> {
        self.orientation = self.imu.as_ref().and_then(CalibratedImu::sample);
        self.orientation
    }

    pub fn orientation(&self) -> Option<Orientation> {
        self.orientation
    }

    pub fn imu_telemetry(&self) -> ([f64; 3], [f64; 3]) {
        self.imu
            .as_ref()
            .map_or(([f64::NAN; 3], [f64::NAN; 3]), CalibratedImu::telemetry)
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

    fn capture_snapshot(&self, drive: [Option<Length>; 2]) -> Snapshot {
        let gps = self.gps.as_ref().and_then(gps_fix).map(|mut fix| {
            fix.pose = fix.pose.in_frame(self.field_origin);
            fix
        });
        let mut snapshot = self.capture_odometry(drive);
        snapshot.gps = gps;
        snapshot
    }

    /// Read only the travel and heading required for low-latency driver odometry.
    /// No range or GPS device calls occur on this path.
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
            gps: None,
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
        (self.capture_snapshot(drive), ranges)
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

fn distance(sensor: &DistanceSensor, occluded: bool) -> Result<Option<RangeObservation>, ()> {
    let before = sensor.timestamp().map_err(|_| ())?;
    let Some(reading) = sensor.object().map_err(|_| ())? else {
        return Ok(None);
    };
    let after = sensor.timestamp().map_err(|_| ())?;
    Ok((before == after).then(|| RangeObservation {
        timestamp: stamp(after),
        distance: Length::new::<millimeter>(f64::from(reading.distance)),
        confidence: reading.confidence,
        occluded,
    }))
}

fn gps_fix(sensor: &GpsSensor) -> Option<GpsFix> {
    let before = sensor.timestamp().ok()?;
    let position = sensor.position().ok()?;
    let heading = compass_heading(UomAngle::new::<radian>(sensor.heading().ok()?.as_radians()));
    let position_error = sensor.error().ok()?;
    let after = sensor.timestamp().ok()?;
    if before != after {
        return None;
    }
    Some(GpsFix {
        pose: Pose::new(
            Length::new::<meter>(position.x),
            Length::new::<meter>(position.y),
            heading,
        ),
        position_error: Length::new::<meter>(position_error),
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
    fn packets_from_before_a_disconnect_do_not_become_ready_again() {
        let mut readiness = RangeReadiness::default();
        let packet = Some((readiness.generation, 7));
        assert_eq!(readiness.current(packet), None);
        readiness.set(true);
        assert_eq!(readiness.current(packet), Some(7));
        readiness.set(false);
        assert_eq!(readiness.current(packet), None);
        readiness.set(true);
        assert_eq!(readiness.current(packet), None);
        let new_packet = Some((readiness.generation, 8));
        assert_eq!(readiness.current(new_packet), Some(8));
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

    #[test]
    fn existing_callback_sources_can_still_be_replaced() {
        let sensors = Sensors::default()
            .with_forward(|| Some(Length::new::<meter>(1.0)))
            .with_forward(|| Some(Length::new::<meter>(2.0)))
            .with_lateral(|| Some(Length::new::<meter>(-3.0)));
        let sample = sensors.capture_odometry([None; 2]);
        assert_eq!(
            (sample.forward, sample.lateral),
            (
                Some(Length::new::<meter>(2.0)),
                Some(Length::new::<meter>(-3.0))
            )
        );
    }
}
