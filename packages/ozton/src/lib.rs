//! Ozton's programmed competition framework.

#![cfg_attr(docsrs, feature(doc_cfg))]

pub use ozton_control as control;
pub use ozton_diagnostics as diagnostics;
pub use ozton_drivetrain as drivetrain;
pub use ozton_localization as localization;
pub use ozton_math as math;
pub use ozton_math::uom;
pub use ozton_motion as motion;
pub use ozton_path_io as path_io;
pub use ozton_runtime as runtime;
pub use ozton_runtime::route;
pub use ozton_tracking::{self as tracking, position, sensor_layout};
pub use ozton_ui as ui;

pub mod prelude {
    pub use crate::{
        control::Tolerances,
        drivetrain::{
            Drivetrain,
            model::{Arcade, ArcadeInput, DriveConfigError, Holonomic, Tank},
        },
        motion::{
            Basic, DriveTuning, PoseSeeking, Seeking,
            commands::{
                Follower, MotionControls, RobotMotion, ScoringControls, TimedMotion,
                TravelDirection,
            },
        },
        runtime::{
            CompetitionBot, Robot, RobotBuilder, RobotLayout,
            competition::{CompetitionLayout, ProgramDrive, Ui},
        },
        tracking::{
            DummyTracking, NoTracking, TracksForwardTravel, TracksHeading, TracksPosition,
            TracksVelocity, WheeledTracking, sources::vex::DistanceTracker,
        },
    };
}
