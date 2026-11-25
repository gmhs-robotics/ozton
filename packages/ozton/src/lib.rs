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
pub use ozton_tracking::{self as tracking, shared_motors};
pub use ozton_ui as ui;

pub mod autonomous;

pub mod prelude {
    pub use crate::{
        autonomous::{ControlError, ControlResult},
        drivetrain::{
            BrakeMode, BrakeMotors, Drive, Drivetrain, HasBraking, HasLocalization, LateralDrive,
            MotorAddress, WheelActuation, WheelCommand, model::DriveConfigError,
        },
        motion::{
            Arcade, Basic, Boomerang, DriveTuning, Holonomic, LtvLqr, PurePursuit, Ramsete,
            Tangent, Tank, TravelDirection, arcade::ArcadeInput, capabilities::*,
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
