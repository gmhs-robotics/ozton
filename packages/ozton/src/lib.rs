//! Ozton's programmed and recording competition framework.

#![cfg_attr(docsrs, feature(doc_cfg))]

pub use ozton_control as control;
#[cfg(feature = "recording")]
pub use ozton_derive as derive;
pub use ozton_diagnostics as diagnostics;
pub use ozton_drivetrain as drivetrain;
pub use ozton_math as math;
pub use ozton_math::uom;
#[cfg(feature = "programmed")]
pub use ozton_motion as motion;
#[cfg(feature = "recording")]
pub use ozton_record as record;
#[cfg(feature = "recording")]
pub use ozton_record::{log, record_frame};
pub use ozton_runtime as runtime;
#[cfg(feature = "programmed")]
pub use ozton_runtime::autonomous_routes;
pub use ozton_tracking::{self as tracking, sensor_layout};

pub mod prelude {
    #[cfg(feature = "programmed")]
    pub use crate::motion::auton::{AutonomousControls, ScoringControls};
    #[cfg(feature = "recording")]
    pub use crate::record::{
        DifferentialCommandFrame, DifferentialPlayback, DifferentialRecording,
        DifferentialVoltagePlayback, Recordable, RecordableDrivetrain, frame::RecordMode,
    };
    #[cfg(feature = "programmed")]
    pub use crate::runtime::{
        Robot, Subsystem, SubsystemList,
        competition::{CompetitionControls, CompetitionRobot},
    };
    pub use crate::{
        control::Tolerances,
        drivetrain::{
            Drivetrain,
            model::{Arcade, Holonomic, Tank},
        },
        tracking::{
            DummyTracking, NoTracking, TracksForwardTravel, TracksHeading, TracksPosition,
            TracksVelocity,
        },
    };
}
