//! # ozton
//!

#![cfg_attr(docsrs, feature(doc_cfg))]

#[doc(inline)]
#[cfg(feature = "control")]
pub use ozton_control as control;
#[doc(inline)]
#[cfg(feature = "derive")]
pub use ozton_derive as derive;
#[doc(inline)]
#[cfg(feature = "drivetrain")]
pub use ozton_drivetrain as drivetrain;
#[doc(inline)]
#[cfg(feature = "math")]
pub use ozton_math as math;
#[doc(inline)]
#[cfg(feature = "motion")]
pub use ozton_motion as motion;
#[doc(inline)]
#[cfg(feature = "record")]
pub use ozton_record as record;
#[cfg(feature = "record")]
pub use ozton_record::log;
#[cfg(feature = "record")]
pub use ozton_record::record_frame;
#[doc(inline)]
#[cfg(feature = "tracking")]
pub use ozton_tracking as tracking;

pub mod prelude {
    #[cfg(feature = "control")]
    pub use crate::control::Tolerances;
    #[cfg(feature = "drivetrain")]
    pub use crate::drivetrain::{
        Drivetrain,
        model::{Arcade, Holonomic, Tank},
    };
    #[cfg(feature = "record")]
    pub use crate::record::{
        DifferentialCommandFrame, DifferentialPlayback, DifferentialRecording,
        DifferentialVoltagePlayback, Recordable, RecordableDrivetrain, RecordableRobot,
        RecordedDifferential, frame::RecordMode,
    };
    #[cfg(feature = "tracking")]
    pub use crate::tracking::{
        DummyTracking, NoTracking, TracksForwardTravel, TracksHeading, TracksPosition,
        TracksVelocity,
    };
}
