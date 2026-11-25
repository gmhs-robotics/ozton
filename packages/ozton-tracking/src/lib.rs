//! Robot localization and tracking.
//!
//! This crate provides traits and systems for tracking and localizing the position, orientation,
//! velocity, and forward travel of a mobile robot in a 2D environment. The goal is to measure
//! accurate data about the robot and provide it to motion algorithms.
//!
//! The traits provided in this crate allow for motion algorithms to operate generically over any
//! tracking system implementation, provided that the implementation measures the necessary data.
//!
//! Tracking systems may choose to implement the following traits for motion algorithms to use:
//!
//! - [`TracksPosition`], for tracking the robot's 2D position (odometry).
//! - [`TracksHeading`], for tracking the robot's absolute orientation (heading).
//! - [`TracksVelocity`], for tracking the robot's linear and angular velocity.
//! - [`TracksForwardTravel`], for tracking the robot's signed forward wheel travel.
//!
//! Below is an example of a motion algorithm function that is generic across all differential
//! drivetrains with tracking systems that measure a robot's forward wheel travel:
//!
//! ```ignore
//! async fn motion(drivetrain: &mut Drivetrain<Differential, impl TrackingSystem, impl TracksForwardTravel>) {
//!     // Motion goes here...
//! }
//! ```
//!
//! Use mounted wheel observations with the localization crate for pose estimation.
//!
//! # Units
//!
//! Use metres, radians, seconds and SI derivatives.

pub mod absolute;
pub mod capabilities;
#[cfg(all(feature = "vex", feature = "logging"))]
#[doc(hidden)]
pub mod hardware;
pub mod inertial;
mod measurement;
pub use measurement::MeasurementSink;
pub use sampling::{RangeBank, Sources, Tracking, TrackingSources};
pub mod odometric;
pub mod range;
pub mod sampling;
pub mod sensor;
pub mod snapshot;

pub use sensor::{ReadPosition, SetPosition};

/// A tracking configuration. Optional reporting is not a motion capability.
pub trait TrackingSystem {
    /// Whether a calibrated differential drive source has been selected.
    const HAS_SELECTED_DRIVE: bool = false;

    /// Observe the explicitly selected source for diagnostics and generic
    /// observation ingestion. Physical wheel commands require HasDriveWheels.
    #[doc(hidden)]
    #[cfg(feature = "vex")]
    fn selected_drive(&self) -> Option<&odometric::DifferentialTracking> {
        None
    }
}

/// A tracking configuration with a statically selected calibrated differential
/// drive-wheel source. Hardware failures remain runtime errors.
#[cfg(feature = "vex")]
pub trait HasDriveWheels: TrackingSystem {
    fn calibrated_drive(&self) -> &odometric::DifferentialTracking;
    fn calibrated_drive_mut(&mut self) -> &mut odometric::DifferentialTracking;
}

/// Explicit no-op tracking type for drivetrains that do not have odometry.
///
/// This keeps the drivetrain type intact while intentionally not implementing any of the motion
/// data traits. Any route-following or feedback motion API that requires tracking information will
/// therefore fail at compile time rather than silently behaving as if real odometry exists.
#[derive(Default, Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub struct NoTracking;

impl TrackingSystem for NoTracking {}
impl TrackingSystem for () {}

pub use ozton_math::localization::observation::{
    DisplacementObservation, HasRange, HeadingObservation, Measurement, Observation, ReadHeading,
    ReadTilt, TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity,
};

pub mod mount;

/// Define sensor IDs, ordered mounts and named device wiring from one layout.
#[macro_export]
macro_rules! sensor_layout {
    ($vis:vis enum $id:ident, $devices:ident => $mounts:ident: $mount:ty {
        $($sensor:ident: $device:ident => $value:expr),+ $(,)?
    }) => {
        /// Sensor IDs in mount-array order.
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        #[repr(usize)]
        $vis enum $id { $($sensor),+ }

        $vis const $mounts: [$mount; [$(stringify!($sensor)),+].len()] = [$($value),+];

        /// One device for every mount in the layout.
        $vis struct $devices<T> {
            $(pub $device: T),+
        }

        impl<T> $devices<T> {
            /// Move the devices into an array in mount order.
            $vis fn into_array(self) -> [T; $mounts.len()] {
                [$(self.$device),+]
            }
        }
    };
}

/// Optional one-time startup for a source that needs calibration or device readiness.
pub type SourceInitialization<'a> = Option<
    std::pin::Pin<
        Box<dyn std::future::Future<Output = Result<(), sampling::TrackingBuildError>> + 'a>,
    >,
>;

/// A source can emit any measurement, regardless of how its hardware works.
pub trait TrackingSource: Sized {
    /// Compile-time hardware interfaces exposed by this source.
    type Capabilities: capabilities::SourceCapabilities<Self>;

    /// Whether this concrete source supplies differential encoder observations.
    const DIFFERENTIAL_ENCODERS: bool = <<Self::Capabilities as capabilities::SourceCapabilities<
        Self,
    >>::Drive as capabilities::Flag>::PRESENT;

    /// Number of guaranteed range slots supplied by this concrete source.
    const RANGE_SLOTS: usize =
        <<Self::Capabilities as capabilities::SourceCapabilities<Self>>::Range as capabilities::Flag>::PRESENT as usize;

    fn initialize(&mut self) -> SourceInitialization<'_> {
        None
    }

    fn sample(&mut self, sink: &mut MeasurementSink<'_>);

    /// Projection of cumulative wheel travel onto chassis right, forward and yaw.
    fn wheel_projection(&self) -> Option<glam::DVec3> {
        None
    }

    /// Field-ranging geometry when this source emits range measurements.
    fn range_mount(&self) -> Option<range::RangeSensorConfig> {
        <Self::Capabilities as capabilities::SourceCapabilities<Self>>::range_mount(self)
    }

    #[doc(hidden)]
    #[cfg(feature = "vex")]
    fn inertial(&self) -> Option<&inertial::InertialSensorTracking> {
        None
    }
}
