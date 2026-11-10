//! Ozton's programmed or recording competition framework.

#![cfg_attr(docsrs, feature(doc_cfg))]

#[cfg(all(feature = "programmed", feature = "recording"))]
compile_error!("Ozton requires exactly one mode. For recording, disable default features.");
#[cfg(not(any(feature = "programmed", feature = "recording")))]
compile_error!("Ozton requires either the programmed or recording feature.");

pub use ozton_control as control;
#[cfg(feature = "recording")]
pub use ozton_derive as derive;
pub use ozton_diagnostics as diagnostics;
pub use ozton_drivetrain as drivetrain;
pub use ozton_math as math;
#[cfg(feature = "programmed")]
pub use ozton_motion as motion;
#[cfg(feature = "recording")]
pub use ozton_record as record;
#[cfg(feature = "recording")]
pub use ozton_record::{log, record_frame};
pub use ozton_runtime as runtime;
pub use ozton_tracking as tracking;

pub mod prelude {
    #[cfg(feature = "recording")]
    pub use crate::record::{
        DifferentialCommandFrame, DifferentialPlayback, DifferentialRecording,
        DifferentialVoltagePlayback, Recordable, RecordableDrivetrain, frame::RecordMode,
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
