//! VEX distance sensors, readiness, and packet freshness.

use std::{
    cell::Cell,
    io::{self, Write},
    time::Duration,
};

use arrayvec::ArrayVec;
use ozton_math::uom::si::{
    f64::{Angle as UomAngle, Length},
    length::millimeter,
};
use vexide::{
    prelude::DistanceSensor,
    smart::{PortError, SmartDevice, SmartPort},
    time::LowResolutionTime,
};

use super::stamp;
use crate::{
    mount::Vec3,
    sources::distance::{MAX_RANGES, RangeObservation, RangeSensorConfig},
};

const RANGE_PERIOD: Duration = Duration::from_millis(33);

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
                })
                .ok()?;
            distances.configs.push(config);
        }
        Some(distances)
    }

    /// Give sensors a shared startup deadline. Unready devices keep their
    /// observation slots and are checked again during capture.
    pub async fn initialize(self, timeout: Duration) -> Self {
        let started = std::time::Instant::now();
        loop {
            let timed_out = started.elapsed() >= timeout;
            for device in &self.devices {
                let status = device.sensor.status();
                let ready = matches!(status, Ok(0x82 | 0x86));
                device.set_ready(ready);
                if timed_out && !ready {
                    let port = device.sensor.port_number();
                    let reason = match status {
                        Ok(0) => "still initializing".to_owned(),
                        Ok(code) => format!("unexpected status {code:#04x}"),
                        Err(PortError::Disconnected { .. }) => "no device connected".to_owned(),
                        Err(PortError::IncorrectDevice { actual, .. }) => {
                            format!("wrong device type ({actual:?})")
                        }
                    };
                    let _ = writeln!(
                        io::stderr().lock(),
                        "warning: distance sensor on port {port} unavailable after {timeout:?} ({reason}); continuing without its readings"
                    );
                }
            }
            if timed_out || self.ready_count() == self.len() {
                return self;
            }
            vexide::time::sleep(RANGE_PERIOD).await;
        }
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
            .is_none_or(|last| time.saturating_sub(last) >= RANGE_PERIOD)
        {
            for (index, device) in self.devices.iter().enumerate() {
                // Runtime status is a connection check. Rejoining never runs calibration.
                device.set_ready(matches!(device.sensor.status(), Ok(0x82 | 0x86)));
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
