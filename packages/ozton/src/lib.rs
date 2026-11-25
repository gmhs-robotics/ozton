//! Ozton's programmed competition framework.

#![cfg_attr(docsrs, feature(doc_cfg))]

pub use ozton_diagnostics as diagnostics;
pub use ozton_drivetrain as drivetrain;
pub use ozton_localization as localization;
pub use ozton_math as math;
pub use ozton_math::{
    control, position_xy, position_xyh, position_xyz, position_xyzh, qty, units, uom,
};
pub use ozton_motion as motion;
pub use ozton_runtime as runtime;
pub use ozton_runtime::{route, run};
pub use ozton_tracking::{self as tracking, sensor_layout, shared_motors};
pub use ozton_ui as ui;

pub mod autonomous;

pub mod prelude {
    pub use crate::{
        autonomous::{ControlError, ControlResult, Follower},
        drivetrain::{Drive, Drivetrain, HasLocalization, LateralDrive, model::DriveConfigError},
        motion::{
            Arcade, DriveTuning, Holonomic, Seeking, Tank, arcade::ArcadeInput,
            seeking::TravelDirection,
        },
        runtime::{
            OztonCompete, Runtime,
            competition::{RobotRoute, Ui},
            drive_assist::{AssistedDrivetrain, DriveAssists},
        },
        tracking::{
            HasDriveWheels, HasRange, Measurement, MeasurementSink, NoTracking, ReadHeading,
            ReadTilt, Sources, Tracking, TrackingSource, TracksForwardTravel, TracksHeading,
            TracksPosition, TracksVelocity, range::DistanceTracker,
        },
    };
}
