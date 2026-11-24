//! Ozton's programmed competition framework.

#![cfg_attr(docsrs, feature(doc_cfg))]

pub use ozton_control as control;
pub use ozton_diagnostics as diagnostics;
pub use ozton_drivetrain as drivetrain;
pub use ozton_localization as localization;
pub use ozton_math as math;
pub use ozton_math::{
    angle, length, position, position_xy, position_xyh, position_xyz, position_xyzh, quantity,
    units, uom,
};
pub use ozton_motion as motion;
pub use ozton_path_io as path_io;
pub use ozton_runtime as runtime;
pub use ozton_runtime::route;
pub use ozton_tracking::{self as tracking, sensor_layout, shared_motors};
pub use ozton_ui as ui;

pub mod autonomous;

pub mod prelude {
    pub use crate::{
        autonomous::{Follower, MotionControls, RobotMotion, ScoringControls, TimedMotion},
        control::Tolerances,
        drivetrain::{Drive, Drivetrain, HasLocalization, LateralDrive, model::DriveConfigError},
        motion::{
            Arcade, Basic, DriveTuning, Holonomic, PoseSeeking, PurePursuit, Seeking, Tank,
            arcade::ArcadeInput,
            seeking::pose::{ResetStartPose, TravelDirection},
        },
        runtime::{
            CompetitionBot, Robot, RobotBuilder, RobotLayout,
            competition::{CompetitionLayout, ProgramDrive, Ui},
            drive_assist::{AssistedDrivetrain, DriveAssists},
        },
        tracking::{
            DummyTracking, Measurement, MeasurementSink, NoTracking, Tracking, TrackingSource,
            TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity, WheeledTracking,
            range::DistanceTracker,
        },
    };
}
