//! GPS-backed field tracking.

use std::{
    cell::RefCell,
    rc::Rc,
    time::{Duration, Instant},
};

use glam::DVec2 as Vec2;
use vexide::{
    math::Angle,
    smart::{PortError, gps::GpsSensor},
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
    pub max_position_error: Option<f64>,
}

impl Default for GpsTrackingConfig {
    fn default() -> Self {
        Self {
            startup_delay: Duration::from_millis(250),
            update_interval: Duration::from_millis(10),
            required_stable_samples: 5,
            max_position_error: None,
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
    position_error: Option<f64>,
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
        let _ = gps.set_data_interval(config.update_interval);

        let data = Rc::new(RefCell::new(GpsTrackingData {
            ..Default::default()
        }));

        Self {
            data: data.clone(),
            _task: spawn(Self::task(gps, config, data)),
        }
    }

    /// Returns the most recent GPS-reported RMS position error in meters.
    pub fn position_error(&self) -> Option<f64> {
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

    fn read_position(gps: &GpsSensor) -> Result<Vec2, PortError> {
        gps.position()
            .map(|position| Vec2::new(position.x, position.y))
    }

    fn read_heading(gps: &GpsSensor) -> Result<Angle, PortError> {
        gps.heading()
            .map(|heading| Angle::from_degrees(90.0 - heading.as_degrees()).wrapped_full())
    }

    fn read_angular_velocity(gps: &GpsSensor) -> Result<f64, PortError> {
        gps.gyro_rate().map(|rate| (-rate.z).to_radians())
    }

    fn is_position_usable(config: GpsTrackingConfig, position_error: Option<f64>) -> bool {
        position_error.is_some_and(|error| {
            config
                .max_position_error
                .is_none_or(|maximum| error <= maximum)
        })
    }

    async fn task(
        gps: GpsSensor,
        config: GpsTrackingConfig,
        data: Rc<RefCell<GpsTrackingData>>,
    ) {
        if !config.startup_delay.is_zero() {
            log!(
                "gps.task: waiting {}ms before first sample",
                config.startup_delay.as_millis()
            );
            sleep(config.startup_delay).await;
        }

        let mut prev_position = Self::read_position(&gps).unwrap_or_default();
        let mut prev_heading = Self::read_heading(&gps).unwrap_or_default();
        let mut prev_time = Instant::now();
        {
            let mut data = data.borrow_mut();
            data.position = prev_position;
            data.heading = prev_heading;
            data.position_error = gps.error().ok();
            data.status = gps.status().ok();
            data.stable_samples = 0;
            data.ready = false;
        }
        log!("gps.task: started");

        loop {
            sleep(config.update_interval).await;

            let dt = prev_time.elapsed();
            prev_time = Instant::now();

            let mut data = data.borrow_mut();
            data.position_error = gps.error().ok();
            data.status = gps.status().ok();
            log!(
                "gps.task: dt={}us raw_error={:?} raw_status={:?}",
                dt.as_micros(),
                data.position_error,
                data.status
            );

            let heading_result = Self::read_heading(&gps);
            let heading = heading_result.unwrap_or(prev_heading);
            let delta_heading = (heading - prev_heading).wrapped_half();
            let dt_seconds = dt.as_secs_f64();

            data.heading = heading;
            data.angular_velocity = if dt_seconds > 0.0 {
                Self::read_angular_velocity(&gps)
                    .unwrap_or_else(|_| delta_heading.as_radians() / dt_seconds)
            } else {
                0.0
            };
            log!(
                "gps.task: heading={:?} delta_heading_rad={:.4} angular_velocity={:.4}",
                data.heading,
                delta_heading.as_radians(),
                data.angular_velocity
            );

            let position_is_usable = Self::is_position_usable(config, data.position_error);
            let position_result = if position_is_usable {
                Self::read_position(&gps).ok()
            } else {
                None
            };
            let sample_is_usable = heading_result.is_ok() && position_result.is_some();

            if let Some(position) = position_result {
                let delta_position = position - prev_position;
                let avg_heading = (prev_heading + (delta_heading / 2.0)).wrapped_full();
                let local_displacement =
                    Vec2::from_angle(-avg_heading.as_radians()).rotate(delta_position);

                data.position = position;
                data.forward_travel += local_displacement.x;
                data.linear_velocity = if dt_seconds > 0.0 {
                    local_displacement.x / dt_seconds
                } else {
                    0.0
                };
                log!(
                    "gps.task: position accepted position={position:?} delta_position={delta_position:?} local_displacement={local_displacement:?} forward_travel={:.4} linear_velocity={:.4}",
                    data.forward_travel,
                    data.linear_velocity
                );

                prev_position = position;
            } else {
                data.linear_velocity = 0.0;
                log!(
                    "gps.task: position rejected usable={} error={:?} max_allowed={:?}",
                    position_is_usable,
                    data.position_error,
                    config.max_position_error
                );
            }

            if sample_is_usable {
                data.stable_samples = data.stable_samples.saturating_add(1);
            } else {
                data.stable_samples = 0;
            }
            data.ready = data.stable_samples >= config.required_stable_samples;
            log!(
                "gps.task: sample_is_usable={} stable_samples={} ready={}",
                sample_is_usable,
                data.stable_samples,
                data.ready
            );

            prev_heading = heading;
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
