//! VEX distance sensors, readiness, and packet freshness.

use std::{cell::Cell, time::Duration};

use arrayvec::ArrayVec;
use futures_util::future::join_all;
use ozton_diagnostics::{log_event, log_row};
use ozton_math::uom::si::{
    f64::{Angle as UomAngle, Length},
    length::millimeter,
};
use vexide::{
    prelude::DistanceSensor,
    smart::{SmartDevice, SmartPort, distance::DistanceReadyError},
    time::LowResolutionTime,
};

use super::stamp;
use crate::{
    mount::Vec3,
    sources::distance::{MAX_RANGES, RangeObservation, RangeSensorConfig},
};

mod compat;
use compat::{ALTERNATE_READY_STATUS, read_object};

// Poll for packet edges faster than the sensor's roughly 33 ms update interval.
const RANGE_POLL_PERIOD: Duration = Duration::from_millis(10);
const MAX_POLL_GAP: Duration = Duration::from_millis(40);

/// An owned distance sensor and its surveyed right/forward/up mounting point.
pub struct DistanceTracker {
    sensor: DistanceSensor,
    config: RangeSensorConfig,
}

impl DistanceTracker {
    /// Bearing is counterclockwise from forward.
    pub fn new(port: SmartPort, position: Vec3<Length>, bearing: UomAngle) -> Self {
        Self::from_sensor(DistanceSensor::new(port), bearing, position)
    }

    pub fn from_sensor(sensor: DistanceSensor, bearing: UomAngle, position: Vec3<Length>) -> Self {
        Self {
            sensor,
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
    last_poll: Option<Duration>,
    last_occluded: u8,
    cached: [Option<(u32, RangeObservation)>; MAX_RANGES],
}

impl Distances {
    /// Register mounts in route index order.
    pub fn new(devices: impl IntoIterator<Item = DistanceTracker>) -> Option<Self> {
        let mut distances = Self::default();
        for DistanceTracker { sensor, config } in devices {
            distances
                .devices
                .try_push(DistanceDevice {
                    sensor,
                    readiness: Cell::default(),
                    clock: PacketClock::default(),
                })
                .ok()?;
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
        let readiness = self.devices.get(index)?.readiness.get();
        let mut observation = readiness.current(self.cached[index])?;
        observation.occluded |= self.last_occluded & (1 << index) != 0;
        Some(observation)
    }

    pub(super) fn capture(&mut self, occluded: u8) -> [Option<RangeObservation>; MAX_RANGES] {
        let time = stamp(LowResolutionTime::now());
        if self
            .last_poll
            .is_none_or(|last| time.saturating_sub(last) >= RANGE_POLL_PERIOD)
        {
            let rebaseline = self
                .last_poll
                .is_none_or(|last| time.saturating_sub(last) > MAX_POLL_GAP);
            for (index, device) in self.devices.iter_mut().enumerate() {
                if rebaseline {
                    device.clock = PacketClock::default();
                }
                let observation = device.capture(time, occluded & (1 << index) != 0);
                if observation.is_err() {
                    device.clock = PacketClock::default();
                }
                log_row!(
                    Trace,
                    "DISTANCE_PACKET",
                    "index={index},port={},read_ok={},object={},occluded={}",
                    device.sensor.port_number(),
                    observation.is_ok(),
                    matches!(observation, Ok(Some(_))),
                    occluded & (1 << index) != 0
                );
                device.set_ready(observation.is_ok());
                let generation = device.readiness.get().generation;
                self.cached[index] = observation.ok().flatten().map(|value| (generation, value));
            }
            self.last_poll = Some(time);
        }
        self.last_occluded = occluded;
        core::array::from_fn(|index| self.latest(index))
    }
}

impl DistanceDevice {
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
                distance: Length::new::<millimeter>(f64::from(millimeters)),
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
