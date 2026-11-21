//! GPS-backed field tracking.

use std::{cell::RefCell, rc::Rc, time::Duration};

use glam::DVec2 as Vec2;
use ozton_math::{
    geometry::Pose,
    uom::si::{f64::Length, length::meter},
};
use vexide::{
    math::Angle,
    smart::{PortError, SmartDevice, gps::GpsSensor},
    task::{Task, spawn},
    time::sleep,
};

use crate::{Tracking, TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity};

/// Configuration for [`GpsTracking`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpsTrackingConfig {
    /// Delay before the tracker starts polling the GPS.
    pub startup_delay: Duration,

    /// Sampling interval for GPS updates.
    pub update_interval: Duration,

    /// Number of consecutive usable samples required before GPS data is treated as reliable.
    pub required_stable_samples: usize,

    /// Maximum acceptable GPS RMS position error in meters.
    ///
    /// Samples with a larger reported error are ignored for position and forward travel updates.
    pub max_position_error: Option<Length>,
    /// Route origin and initial facing in the centered GPS survey frame.
    pub field_origin: Pose,
}

impl Default for GpsTrackingConfig {
    fn default() -> Self {
        Self {
            startup_delay: Duration::from_millis(250),
            update_interval: Duration::from_millis(10),
            required_stable_samples: 5,
            max_position_error: None,
            field_origin: Pose::default(),
        }
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
struct GpsTrackingData {
    position: Vec2,
    heading: Angle,
    forward_travel: f64,
    linear_velocity: f64,
    angular_velocity: f64,
    position_error: Option<Length>,
    status: Option<u32>,
    stable_samples: usize,
    ready: bool,
}

/// Tracking system that derives differential-drive odometry primitives from a [`GpsSensor`].
///
/// This tracker converts absolute field coordinates from the sensor into the trait surface used by
/// ozton's motion algorithms, including signed forward travel and velocity estimation.
#[derive(Debug)]
pub struct GpsTracking {
    data: Rc<RefCell<GpsTrackingData>>,
    _task: Task<()>,
}

impl GpsTracking {
    /// Creates a GPS tracker that uses the `GpsSensor`'s native meter-based position readings.
    pub fn new(gps: GpsSensor) -> Self {
        log!("gps.new: using default config");
        Self::with_config(gps, GpsTrackingConfig::default())
    }

    /// Creates a GPS tracker with explicit configuration.
    pub fn with_config(mut gps: GpsSensor, config: GpsTrackingConfig) -> Self {
        log!(
            "gps.with_config: startup_delay={}ms update_interval={}ms required_stable_samples={} max_position_error={:?}",
            config.startup_delay.as_millis(),
            config.update_interval.as_millis(),
            config.required_stable_samples,
            config.max_position_error
        );
        if let Err(error) = gps.set_data_interval(config.update_interval) {
            log!("gps.with_config: device interval rejected: {error:?}");
        }

        let data = Rc::new(RefCell::new(GpsTrackingData {
            ..Default::default()
        }));

        Self {
            data: data.clone(),
            _task: spawn(Self::task(gps, config, data)),
        }
    }

    /// Returns the most recent GPS-reported RMS position error in meters.
    pub fn position_error(&self) -> Option<Length> {
        self.data.borrow().position_error
    }

    /// Returns the most recent raw GPS status word.
    pub fn status(&self) -> Option<u32> {
        self.data.borrow().status
    }

    /// Returns whether the tracker has accumulated enough usable samples to trust its pose.
    pub fn is_ready(&self) -> bool {
        self.data.borrow().ready
    }

    /// Returns the current count of consecutive usable samples.
    pub fn stable_samples(&self) -> usize {
        self.data.borrow().stable_samples
    }

    fn read_position(gps: &GpsSensor, origin: Pose) -> Result<Vec2, PortError> {
        gps.position()
            .map(|position| origin.to_local(Vec2::new(position.x, position.y)))
    }

    fn read_heading(gps: &GpsSensor, origin: Pose) -> Result<Angle, PortError> {
        gps.heading().map(|heading| {
            (Angle::from_degrees(-heading.as_degrees()) - Angle::from_radians(origin.heading))
                .wrapped_full()
        })
    }

    fn read_angular_velocity(gps: &GpsSensor) -> Result<f64, PortError> {
        gps.gyro_rate().map(|rate| (-rate.z).to_radians())
    }

    fn is_position_usable(config: GpsTrackingConfig, position_error: Option<Length>) -> bool {
        position_error.is_some_and(|error| {
            error.value.is_finite()
                && error.value >= 0.0
                && config
                    .max_position_error
                    .is_none_or(|maximum| error <= maximum)
        })
    }

    async fn task(gps: GpsSensor, config: GpsTrackingConfig, data: Rc<RefCell<GpsTrackingData>>) {
        if !config.field_origin.is_finite() {
            log!("gps.task: invalid field origin; tracking remains unavailable");
            return;
        }
        if !config.startup_delay.is_zero() {
            log!(
                "gps.task: waiting {}ms before first sample",
                config.startup_delay.as_millis()
            );
            sleep(config.startup_delay).await;
        }

        let interval = config.update_interval.max(Duration::from_millis(1));
        let max_gap = interval.saturating_mul(3).max(Duration::from_millis(100));
        let mut previous = None;
        let mut last_packet = None;
        let mut since_packet = std::time::Instant::now();

        loop {
            sleep(interval).await;
            let packet = (|| {
                let timestamp = gps.timestamp().ok()?;
                let position_error = gps.error().ok()?;
                let position = Self::read_position(&gps, config.field_origin).ok()?;
                let heading = Self::read_heading(&gps, config.field_origin).ok()?;
                let rate = Self::read_angular_velocity(&gps)
                    .ok()
                    .filter(|rate| rate.is_finite());
                let status = gps.status().ok();
                let after = gps.timestamp().ok()?;
                (timestamp == after && position.is_finite() && heading.as_radians().is_finite())
                    .then_some((
                        timestamp,
                        position,
                        heading,
                        Length::new::<meter>(position_error),
                        rate,
                        status,
                    ))
            })();
            if packet
                .as_ref()
                .is_some_and(|(time, ..)| last_packet == Some(*time))
                && since_packet.elapsed() <= max_gap
            {
                continue;
            }
            let mut state = data.borrow_mut();
            let Some((time, position, heading, error, rate, status)) = packet else {
                previous = None;
                state.ready = false;
                state.stable_samples = 0;
                state.linear_velocity = 0.0;
                state.angular_velocity = 0.0;
                continue;
            };
            if last_packet.is_some_and(|last| time <= last) {
                previous = None;
                state.ready = false;
                state.stable_samples = 0;
                state.linear_velocity = 0.0;
                state.angular_velocity = 0.0;
                continue;
            }
            last_packet = Some(time);
            since_packet = std::time::Instant::now();
            state.position_error = Some(error);
            state.status = status;
            if !Self::is_position_usable(config, Some(error)) {
                previous = None;
                state.ready = false;
                state.stable_samples = 0;
                state.linear_velocity = 0.0;
                state.angular_velocity = 0.0;
                continue;
            }

            state.position = position;
            state.heading = heading;
            state.linear_velocity = 0.0;
            state.angular_velocity = rate.unwrap_or(0.0);
            if let Some((last_time, last_position, last_heading)) =
                previous.replace((time, position, heading))
            {
                let span = time.duration_since(last_time);
                if !span.is_zero() && span <= max_gap {
                    let delta_heading = (heading - last_heading).wrapped_half();
                    let midpoint = last_heading + delta_heading / 2.0;
                    let delta = Vec2::from_angle(-midpoint.as_radians())
                        .rotate(position - last_position)
                        .y;
                    state.forward_travel += delta;
                    state.linear_velocity = delta / span.as_secs_f64();
                    state.angular_velocity =
                        rate.unwrap_or(delta_heading.as_radians() / span.as_secs_f64());
                } else {
                    state.stable_samples = 0;
                }
            }
            state.stable_samples = state.stable_samples.saturating_add(1);
            state.ready = state.stable_samples >= config.required_stable_samples.max(1);
        }
    }
}

impl Tracking for GpsTracking {}

impl TracksPosition for GpsTracking {
    fn position(&self) -> Vec2 {
        self.data.borrow().position
    }
}

impl TracksHeading for GpsTracking {
    fn heading(&self) -> Angle {
        self.data.borrow().heading
    }
}

impl TracksVelocity for GpsTracking {
    fn linear_velocity(&self) -> f64 {
        self.data.borrow().linear_velocity
    }

    fn angular_velocity(&self) -> f64 {
        self.data.borrow().angular_velocity
    }
}

impl TracksForwardTravel for GpsTracking {
    fn forward_travel(&self) -> f64 {
        self.data.borrow().forward_travel
    }
}
