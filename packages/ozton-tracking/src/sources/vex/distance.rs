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
    smart::{PortError, SmartDevice, SmartPort, distance::DistanceReadyError},
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

    /// Wait for each sensor to finish initializing. Devices that return errors
    /// keep their observation slots and are checked again during capture.
    pub async fn initialize(self) -> Self {
        for device in &self.devices {
            let port = device.sensor.port_number();
            let _ = writeln!(
                io::stderr().lock(),
                "info: waiting for distance sensor on port {port}"
            );
            let ready = device.sensor.wait_ready().await;
            device.set_ready(ready.is_ok());
            match ready {
                Ok(()) => {
                    let _ = writeln!(
                        io::stderr().lock(),
                        "info: distance sensor on port {port} ready"
                    );
                }
                Err(error) => {
                    let reason = match error {
                        DistanceReadyError::UnknownStatusCode { code } => {
                            format!("device detected, unrecognized status {code:#04x}")
                        }
                        DistanceReadyError::Port {
                            source: PortError::Disconnected { .. },
                        } => "no device connected".to_owned(),
                        DistanceReadyError::Port {
                            source: PortError::IncorrectDevice { actual, .. },
                        } => {
                            format!("wrong device type ({actual:?})")
                        }
                    };
                    let _ = writeln!(
                        io::stderr().lock(),
                        "warning: distance sensor on port {port} unavailable ({reason}); continuing without its readings"
                    );
                }
            }
        }
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
            .is_none_or(|last| time.saturating_sub(last) >= RANGE_PERIOD)
        {
            for (index, device) in self.devices.iter().enumerate() {
                let observation = distance(&device.sensor, occluded & (1 << index) != 0);
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
