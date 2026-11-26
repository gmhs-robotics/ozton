//! Ozton's programmed competition framework.

#![cfg_attr(docsrs, feature(doc_cfg))]

pub use ozton_assist as assist;
pub use ozton_diagnostics as diagnostics;
pub use ozton_drivetrain as drivetrain;
pub use ozton_localization as localization;
pub use ozton_math as math;
pub use ozton_math::{
    control, position_xy, position_xyh, position_xyz, position_xyzh, qty, units, uom,
};
pub use ozton_motion as motion;
pub use ozton_runtime as runtime;
pub use ozton_runtime::{route, run, run_with_metadata};
pub use ozton_tracking::{self as tracking, shared_motors};
pub use ozton_ui as ui;

pub mod autonomous;

pub mod prelude {
    pub use crate::{
        assist::{CurvatureDrive, DriveAssist, DriveAssists},
        autonomous::{ControlError, ControlResult},
        drivetrain::{
            BrakeMode, BrakeMotors, Drive, Drivetrain, HasBraking, HasLocalization, LateralDrive,
            MotorAddress, WheelActuation, WheelCommand, model::DriveConfigError,
        },
        motion::{
            Basic, Boomerang, DriveTuning, LtvLqr, PurePursuit, Ramsete, RunMotion,
            TravelDirection, capabilities::*,
        },
        route,
        runtime::{
            OztonCompete, Runtime,
            competition::{RobotRoute, Ui},
        },
        tracking::{
            HasDriveWheels, HasRange, Measurement, MeasurementSink, NoTracking, ReadHeading,
            ReadTilt, Sources, Tracking, TrackingSource, TracksForwardTravel, TracksHeading,
            TracksPosition, TracksVelocity, range::DistanceTracker,
        },
    };
}
