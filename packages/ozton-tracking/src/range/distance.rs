//! VEX distance sensors, readiness, and packet freshness.

use std::time::Duration;

use ozton_diagnostics::log_event;
#[cfg(feature = "logging")]
use ozton_diagnostics::log_row;
use ozton_math::uom::si::f64::{Angle as UomAngle, Length};
use vexide::{prelude::DistanceSensor, smart::SmartDevice, time::LowResolutionTime};

use crate::{
    mount::Vec3,
    range::{RangeObservation, RangeRoles, RangeSensorConfig},
};

mod compat;
use compat::read_object;

// Poll for packet edges faster than the sensor's roughly 33 ms update interval.
const RANGE_POLL_PERIOD: Duration = Duration::from_millis(10);
const MAX_POLL_GAP: Duration = Duration::from_millis(40);

/// An owned distance sensor and its surveyed right/forward/up mounting point.
pub struct DistanceTracker {
    sensor: DistanceSensor,
    clock: PacketClock,
    last_poll: Option<Duration>,
    #[cfg(feature = "logging")]
    packet_report: PacketReport,
    latency: Duration,
    cached: Option<RangeObservation>,
    config: RangeSensorConfig,
}

impl DistanceTracker {
    /// Mount the device at a surveyed XYZ position and beam heading.
    pub fn at_mount(sensor: DistanceSensor, (position, bearing): (Vec3<Length>, UomAngle)) -> Self {
        Self {
            sensor,
            clock: PacketClock::default(),
            last_poll: None,
            #[cfg(feature = "logging")]
            packet_report: PacketReport::default(),
            latency: Duration::ZERO,
            cached: None,
            config: RangeSensorConfig::at_mount((position, bearing)),
        }
    }

    /// Correct a characterized sensor transport/acquisition delay.
    #[must_use]
    pub fn with_latency(mut self, latency: Duration) -> Self {
        self.latency = latency;
        self
    }

    /// Correct a measured systematic range offset (subtracted from readings).
    #[must_use]
    pub fn with_bias(mut self, bias: Length) -> Self {
        self.config.bias = bias;
        self
    }

    /// Set the calibrated minimum measurement standard deviation for this sensor.
    /// Published distance-dependent accuracy remains a lower bound.
    #[must_use]
    pub fn with_sigma(mut self, sigma: Length) -> Self {
        self.config.sigma = sigma;
        self
    }

    /// Limit accepted ranges to a caller-selected maximum approximate error.
    /// Invalid or impossible budgets return None rather than silently broadening the gate.
    #[must_use]
    pub fn with_maximum_range_error(mut self, error: Length) -> Option<Self> {
        self.config.limit_accuracy(error).then_some(self)
    }

    /// Choose which localization algorithms may use this sensor.
    #[must_use]
    pub fn with_roles(mut self, roles: RangeRoles) -> Self {
        self.config = self.config.with_roles(roles);
        self
    }

    /// Override the measurement acceptance gate for this sensor.
    #[must_use]
    pub fn with_gate(mut self, gate: ozton_math::localization::range::RangeGate) -> Self {
        self.config.gate = gate;
        self
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

#[cfg(feature = "logging")]
#[derive(Default)]
struct PacketReport {
    last: Option<(Duration, [bool; 3])>,
}

#[cfg(feature = "logging")]
impl PacketReport {
    fn due(&mut self, time: Duration, state: [bool; 3]) -> bool {
        const HEARTBEAT: Duration = Duration::from_millis(100);
        if self.last.is_none_or(|(last, previous)| {
            state != previous || time.checked_sub(last).is_none_or(|age| age >= HEARTBEAT)
        }) {
            self.last = Some((time, state));
            true
        } else {
            false
        }
    }
}

impl super::RangeSource for DistanceTracker {
    fn range_config(&self) -> RangeSensorConfig {
        self.config
    }
}

impl crate::TrackingSource for DistanceTracker {
    type Capabilities = crate::capabilities::Range;

    // Range sources are optional pose observations. A disconnected or slowly
    // booting sensor must not delay Tracking::new. Runtime packet validation
    // establishes readiness and requires a fresh packet before emitting data.
    fn sample(&mut self, sink: &mut crate::MeasurementSink<'_>) {
        self.poll(
            sink.time,
            sink.range_occluded,
            sink.range_index.unwrap_or(sink.source),
        );
        if let Some(range) = self.latest(sink.range_occluded) {
            sink.push(crate::Measurement::Range(range));
        }
    }
}

impl DistanceTracker {
    fn latest(&self, occluded: bool) -> Option<RangeObservation> {
        let mut observation = self.cached?;
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
        #[cfg(feature = "logging")]
        if ozton_diagnostics::serial::sd_enabled() {
            let state = [
                observation.is_ok(),
                matches!(observation, Ok(Some(_))),
                occluded,
            ];
            if self.packet_report.due(time, state) {
                log_row!(
                    Trace,
                    "DISTANCE_PACKET",
                    "index={_index},port={},read_ok={},object={},occluded={occluded}",
                    self.sensor.port_number(),
                    state[0],
                    state[1]
                );
            }
        }
        self.cached = observation.ok().flatten();
        self.last_poll = Some(time);
    }

    fn capture(&mut self, now: Duration, occluded: bool) -> Result<Option<RangeObservation>, ()> {
        let before = self.sensor.timestamp().map_err(|_| ())?;
        // The Brain refreshes its packet cache every 10 ms, but the range
        // sensor produces measurements at about 30 Hz. Inspect packet edges
        // without repeatedly decoding the same measurement.
        if self.clock.last.is_some_and(|(packet, _)| packet == before) {
            return Ok(self.cached);
        }
        let object = read_object(&self.sensor)?;
        let after = self.sensor.timestamp().map_err(|_| ())?;
        if before != after {
            return Ok(None);
        }
        // A new packet arrived somewhere between the previous poll and
        // this one. Use the midpoint, then remove characterized transport lag.
        let arrival = self
            .last_poll
            .map_or(now, |last| last + now.saturating_sub(last) / 2);
        let timestamp = self
            .clock
            .observe(after, arrival.saturating_sub(self.latency));
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

    #[cfg(feature = "logging")]
    #[test]
    fn packet_report_preserves_edges_and_periodic_heartbeat() {
        let mut report = PacketReport::default();
        let ms = Duration::from_millis;
        let healthy = [true, true, false];
        let failed = [false, false, false];
        assert!(report.due(ms(0), healthy));
        assert!(!report.due(ms(10), healthy));
        assert!(report.due(ms(20), failed));
        assert!(report.due(ms(30), healthy));
        assert!(!report.due(ms(129), healthy));
        assert!(report.due(ms(130), healthy));
        assert!(report.due(ms(0), healthy));
    }

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
