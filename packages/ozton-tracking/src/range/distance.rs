//! VEX distance sensors, readiness, and packet freshness.

use std::{cell::Cell, time::Duration};

use arrayvec::ArrayVec;
use futures_util::future::join_all;
use ozton_diagnostics::{log_event, log_row};
use ozton_math::uom::si::f64::{Angle as UomAngle, Length};
use vexide::{
    prelude::DistanceSensor,
    smart::{SmartDevice, distance::DistanceReadyError},
    time::LowResolutionTime,
};

use crate::{
    mount::Vec3,
    range::{MAX_RANGES, RangeObservation, RangeSensorConfig},
    sampling::stamp,
};

mod compat;
use compat::{ALTERNATE_READY_STATUS, read_object};

// Poll for packet edges faster than the sensor's roughly 33 ms update interval.
const RANGE_POLL_PERIOD: Duration = Duration::from_millis(10);
const MAX_POLL_GAP: Duration = Duration::from_millis(40);

/// An owned distance sensor and its surveyed right/forward/up mounting point.
pub struct DistanceTracker {
    device: DistanceDevice,
    config: RangeSensorConfig,
}

impl DistanceTracker {
    /// Mount the device at a surveyed XYZ position and beam heading.
    pub fn at_mount(sensor: DistanceSensor, (position, bearing): (Vec3<Length>, UomAngle)) -> Self {
        Self {
            device: DistanceDevice {
                sensor,
                readiness: Cell::default(),
                clock: PacketClock::default(),
                last_poll: None,
                cached: None,
            },
            config: RangeSensorConfig::at_mount((position, bearing)),
        }
    }

    /// Correct a measured systematic range offset (subtracted from readings).
    #[must_use]
    pub fn with_bias(mut self, bias: Length) -> Self {
        self.config.bias = bias;
        self
    }

    /// Set measurement standard deviation for this sensor.
    #[must_use]
    pub fn with_sigma(mut self, sigma: Length) -> Self {
        self.config.sigma = sigma;
        self
    }

    /// Override the measurement acceptance gate for this sensor.
    #[must_use]
    pub fn with_gate(mut self, gate: ozton_math::localization::range::RangeGate) -> Self {
        self.config.gate = gate;
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

/// Device timestamps identify packets. On the Brain their epoch can drift
/// relative to odometry, so timestamp observed arrivals on the odometry clock.
#[derive(Default)]
struct PacketClock {
    last: Option<(LowResolutionTime, Option<Duration>)>,
}

impl PacketClock {
    fn observe(&mut self, packet: LowResolutionTime, now: Duration) -> Option<Duration> {
        let time = match self.last {
            None => None,
            Some((previous, time)) if previous == packet => time,
            Some(_) => Some(now),
        };
        self.last = Some((packet, time));
        time
    }
}

struct DistanceDevice {
    sensor: DistanceSensor,
    readiness: Cell<RangeReadiness>,
    clock: PacketClock,
    last_poll: Option<Duration>,
    cached: Option<(u32, RangeObservation)>,
}

impl DistanceDevice {
    fn set_ready(&self, ready: bool) {
        let mut readiness = self.readiness.get();
        readiness.set(ready);
        self.readiness.set(readiness);
    }

    async fn initialize(&self) {
        log_event!(
            Info,
            "distance",
            "Waiting for sensor on port {}",
            self.sensor.port_number()
        );
        let ready = match self.sensor.wait_ready().await {
            Err(DistanceReadyError::UnknownStatusCode {
                code: ALTERNATE_READY_STATUS,
            }) => Ok(()),
            result => result,
        };
        self.set_ready(ready.is_ok());
        match ready {
            Ok(()) => log_event!(
                Info,
                "distance",
                "Sensor on port {} ready",
                self.sensor.port_number()
            ),
            Err(error) => log_event!(
                Warn,
                "distance",
                "Sensor on port {} unavailable ({error}). Continuing without its readings.",
                self.sensor.port_number()
            ),
        }
    }
}

/// Distance devices polled once per packet window and shared by higher-level
/// consumers through the cached observations.
#[derive(Default)]
pub struct Distances {
    devices: ArrayVec<DistanceDevice, MAX_RANGES>,
    configs: ArrayVec<RangeSensorConfig, MAX_RANGES>,
    last_occluded: u8,
}

impl Distances {
    /// Register mounts in route index order.
    pub fn new(devices: impl IntoIterator<Item = DistanceTracker>) -> Option<Self> {
        let mut distances = Self::default();
        for DistanceTracker { device, config } in devices {
            distances.devices.try_push(device).ok()?;
            distances.configs.push(config);
        }
        Some(distances)
    }

    /// Wait for each sensor to finish initializing. Devices that return errors
    /// keep their observation slots and are checked again during capture.
    pub async fn initialize(self) -> Self {
        join_all(self.devices.iter().map(DistanceDevice::initialize)).await;
        self
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
        self.devices
            .get(index)?
            .latest(self.last_occluded & (1 << index) != 0)
    }

    pub(crate) fn capture(&mut self, occluded: u8) -> [Option<RangeObservation>; MAX_RANGES] {
        let time = stamp(LowResolutionTime::now());
        for (index, device) in self.devices.iter_mut().enumerate() {
            device.poll(time, occluded & (1 << index) != 0, index);
        }
        self.last_occluded = occluded;
        core::array::from_fn(|index| self.latest(index))
    }
}

impl crate::TrackingSource for DistanceTracker {
    // Range sources are optional pose observations. A disconnected or slowly
    // booting sensor must not delay Tracking::new. Runtime packet validation
    // establishes readiness and requires a fresh packet before emitting data.
    fn sample(&mut self, sink: &mut crate::MeasurementSink<'_>) {
        self.device.poll(
            sink.time,
            sink.range_occluded,
            sink.range_index.unwrap_or(sink.source),
        );
        if let Some(range) = self.device.latest(sink.range_occluded) {
            sink.push(crate::Measurement::Range(range));
        }
    }

    fn range_mount(&self) -> Option<RangeSensorConfig> {
        Some(self.config)
    }
}

impl DistanceDevice {
    fn latest(&self, occluded: bool) -> Option<RangeObservation> {
        let mut observation = self.readiness.get().current(self.cached)?;
        observation.occluded |= occluded;
        Some(observation)
    }

    fn poll(&mut self, time: Duration, occluded: bool, _index: usize) {
        if self
            .last_poll
            .is_some_and(|last| time.saturating_sub(last) < RANGE_POLL_PERIOD)
        {
            return;
        }
        if self
            .last_poll
            .is_none_or(|last| time.saturating_sub(last) > MAX_POLL_GAP)
        {
            self.clock = PacketClock::default();
        }
        let first_poll = self.last_poll.is_none();
        let observation = self.capture(time, occluded);
        if first_poll {
            log_event!(
                Info,
                "distance",
                "Port {} initial packet probe: {}",
                self.sensor.port_number(),
                if observation.is_ok() {
                    "responsive"
                } else {
                    "not ready"
                }
            );
        }
        if observation.is_err() {
            self.clock = PacketClock::default();
        }
        log_row!(
            Trace,
            "DISTANCE_PACKET",
            "index={_index},port={},read_ok={},object={},occluded={occluded}",
            self.sensor.port_number(),
            observation.is_ok(),
            matches!(observation, Ok(Some(_)))
        );
        self.set_ready(observation.is_ok());
        let generation = self.readiness.get().generation;
        self.cached = observation.ok().flatten().map(|value| (generation, value));
        self.last_poll = Some(time);
    }

    fn capture(&mut self, now: Duration, occluded: bool) -> Result<Option<RangeObservation>, ()> {
        let before = self.sensor.timestamp().map_err(|_| ())?;
        let object = read_object(&self.sensor)?;
        let after = self.sensor.timestamp().map_err(|_| ())?;
        if before != after {
            return Ok(None);
        }
        let timestamp = self.clock.observe(after, now);
        Ok(timestamp
            .zip(object)
            .map(|(timestamp, (millimeters, confidence))| RangeObservation {
                timestamp,
                distance: ozton_math::qty!((f64::from(millimeters)) mm),
                confidence,
                occluded,
            }))
    }
}

#[cfg(test)]
mod packet_tests {
    use super::*;

    #[test]
    fn changing_packets_use_odometry_time_and_cached_packets_keep_their_age() {
        let mut clock = PacketClock::default();
        let packet = LowResolutionTime::from_millis_since_epoch;
        let ms = Duration::from_millis;
        // Recorded offsets drifted from ~280 to >400 ms while packets advanced.
        assert_eq!(clock.observe(packet(22_380), ms(22_660)), None);
        assert_eq!(clock.observe(packet(22_415), ms(22_690)), Some(ms(22_690)));
        assert_eq!(clock.observe(packet(22_415), ms(22_710)), Some(ms(22_690)));
        assert_eq!(clock.observe(packet(34_765), ms(35_095)), Some(ms(35_095)));
        assert_eq!(clock.observe(packet(34_765), ms(35_195)), Some(ms(35_095)));
    }
}
