//! Wheeled odometry.

use std::{cell::RefCell, rc::Rc, time::Instant};

use glam::DVec2 as Vec2;
use ozton_math::uom::si::f64::Angle as Heading;
use vexide::{
    math::Angle,
    smart::motor::Motor,
    task::{Task, spawn},
    time::sleep,
};

use crate::{
    TrackingSystem, TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity,
    sensor::{Gyro, RotarySensor},
};

// MARK: Tracking Wheel

mod odometry;
use super::wheel::TrackingWheel;

enum HeadingError<T: RotarySensor> {
    /// Gyro failed for whatever reason (we don't care exactly what happened,
    /// since all gyro failures are potentially fatal to tracking).
    ///
    /// A backup angle computed from two parallel forward tracking is provided,
    /// if such wheels are available.
    Imu(Option<Angle>),

    /// [`RotarySensor`] failed to return position data required to compute heading
    /// from a pair of parallel forward tracking wheels.
    RotarySensor(T::Error),
}

/// Generic tracking data returned by [`ParallelWheelTracking`] and [`PerpendicularWheelTracking`].
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub(crate) struct TrackingData {
    ready: bool,
    position: Vec2,
    raw_heading: Angle,
    heading_offset: Angle,
    forward_travel: f64,
    linear_velocity: f64,
    angular_velocity: f64,
}

// MARK: Tracking Implementation

/// Tracking system that uses wheels to track position and orientation.
#[derive(Debug)]
pub struct WheeledTracking {
    data: Rc<RefCell<TrackingData>>,
    _task: Task<()>,
}

impl WheeledTracking {
    /// Creates a new wheeled tracking system.
    pub fn new<
        T: RotarySensor + 'static,
        U: RotarySensor + 'static,
        G: Gyro + 'static,
        const NUM_FORWARD: usize,
        const NUM_SIDEWAYS: usize,
    >(
        origin: impl Into<Vec2>,
        heading: Angle,
        forward_wheels: [TrackingWheel<T>; NUM_FORWARD],
        sideways_wheels: [TrackingWheel<U>; NUM_SIDEWAYS],
        mut gyro: Option<G>,
    ) -> Self {
        const FORWARD_TRACKER_OFFSET_TOLERANCE: f64 = 0.5;

        const {
            assert!(
                NUM_FORWARD > 0,
                "Wheeled tracking requires at least one forward tracking wheel."
            );
        }

        assert!(
            NUM_FORWARD >= 2 || gyro.is_some(),
            "Wheeled tracking requires either a Gyro or two parallel forward tracking wheels to determine robot orientation."
        );

        // Locate two parallel tracking wheels with roughly the same absolute offset from the
        // center of tracking. We use these for our backup wheeled heading calculations.
        let mut parallel_forward_indicies = None;
        if NUM_FORWARD >= 2 {
            for i in 0..NUM_FORWARD {
                for j in (i + 1)..NUM_FORWARD {
                    let i_wheel = &forward_wheels[i];
                    let j_wheel = &forward_wheels[j];

                    // Check if their offsets are acceptable enough
                    if (i_wheel.offset.value + j_wheel.offset.value).abs()
                        <= FORWARD_TRACKER_OFFSET_TOLERANCE
                        && (i_wheel.offset.value - j_wheel.offset.value).abs() > f64::EPSILON
                    {
                        parallel_forward_indicies =
                            Some(if i_wheel.offset.value < j_wheel.offset.value {
                                (i, j)
                            } else {
                                (j, i)
                            });
                    }
                }
            }
        }

        assert!(
            gyro.is_some() || parallel_forward_indicies.is_some(),
            "No gyro provided or viable tracking wheels available to determine robot orientation."
        );

        let initial_forward_wheel_data = forward_wheels.each_ref().map(|wheel| {
            wheel
                .travel()
                .map(|travel| (travel.value, wheel.offset.value))
        });
        let initial_sideways_wheel_data = sideways_wheels.each_ref().map(|wheel| {
            wheel
                .travel()
                .map(|travel| (travel.value, wheel.offset.value))
        });
        let initial_heading_result = Self::compute_raw_heading(
            gyro.as_ref(),
            parallel_forward_indicies.map(|(left_index, right_index)| {
                (&forward_wheels[left_index], &forward_wheels[right_index])
            }),
        );
        let initially_ready = initial_heading_result.is_ok()
            && initial_forward_wheel_data.iter().all(Result::is_ok)
            && initial_sideways_wheel_data.iter().all(Result::is_ok);
        let initial_raw_heading = match initial_heading_result {
            Ok(heading) => heading,
            Err(HeadingError::Imu(wheel_heading)) => {
                if wheel_heading.is_some() {
                    gyro = None;
                }
                // NOTE: This returning `None` means that there's no real point in spawning the task
                // since the gyro disconnected, but we'll leave that to the task to figure out,
                // since there's an early return condition in the loop if this occurs and we don't
                // want to have to handle errors in this specific function.
                wheel_heading.unwrap_or_default()
            }
            _ => Angle::default(),
        };
        let data = Rc::new(RefCell::new(TrackingData {
            ready: initially_ready,
            position: origin.into(),
            heading_offset: heading - initial_raw_heading,
            raw_heading: initial_raw_heading,
            ..Default::default()
        }));

        Self {
            data: data.clone(),
            _task: spawn(Self::task(
                forward_wheels,
                sideways_wheels,
                gyro,
                data,
                parallel_forward_indicies,
                initial_forward_wheel_data,
                initial_sideways_wheel_data,
                initial_raw_heading,
            )),
        }
    }

    /// Creates a new wheeled tracking system with no sideways tracking wheels.
    pub fn forward_only<T: RotarySensor + 'static, G: Gyro + 'static, const NUM_FORWARD: usize>(
        origin: impl Into<Vec2>,
        heading: Angle,
        forward_wheels: [TrackingWheel<T>; NUM_FORWARD],
        gyro: Option<G>,
    ) -> Self {
        Self::new(
            origin,
            heading,
            forward_wheels,
            [] as [TrackingWheel<T>; 0],
            gyro,
        )
    }

    /// Reports whether tracking has a continuous position estimate.
    pub fn is_ready(&self) -> bool {
        self.data.borrow().ready
    }

    fn aligned_heading_offset(offset: Angle, previous_raw: Angle, new_raw: Angle) -> Angle {
        offset + previous_raw - new_raw
    }

    /// Offsets the current heading to the supplied angle.
    pub fn set_heading(&mut self, heading: Heading) {
        let mut data = self.data.borrow_mut();
        data.heading_offset = Angle::from_radians(heading.value) - data.raw_heading;
    }

    /// Sets the currently tracked position to a new point.
    pub fn set_position(&mut self, position: Vec2) {
        self.data.borrow_mut().position = position;
    }
}

// MARK: Tracking Traits

impl TrackingSystem for WheeledTracking {}

impl TracksPosition for WheeledTracking {
    fn position(&self) -> Vec2 {
        self.data.borrow().position
    }
}

impl TracksHeading for WheeledTracking {
    fn heading(&self) -> Heading {
        let data = self.data.borrow();

        // Apply heading offset and wrap from [0, 2π]
        ozton_math::angle!(
            (data.raw_heading + data.heading_offset)
                .wrapped_full()
                .as_radians(),
            rad,
        )
    }
}

impl TracksForwardTravel for WheeledTracking {
    fn forward_travel(&self) -> f64 {
        self.data.borrow().forward_travel
    }
}

impl TracksVelocity for WheeledTracking {
    fn angular_velocity(&self) -> f64 {
        self.data.borrow().angular_velocity
    }

    fn linear_velocity(&self) -> f64 {
        self.data.borrow().linear_velocity
    }
}
