#![feature(trait_alias)]

pub use async_trait::async_trait;
pub use rkyv;
pub use vexide::smart::PortError;

/// Emits diagnostic traces in debug builds only.
///
/// Arguments are type-checked in every profile, but evaluated only when debug assertions are enabled.
#[macro_export]
macro_rules! log {
    ($($arg:tt)*) => {
        if ::core::cfg!(debug_assertions) {
            ::std::println!("[ozton] {}", ::core::format_args!($($arg)*));
        }
    };
}

// Failures handled here rather than returned to a caller must remain visible in release builds.
macro_rules! log_error {
    ($($arg:tt)*) => {
        ::std::println!("[ozton] error: {}", ::core::format_args!($($arg)*))
    };
}
pub(crate) use log_error;

#[macro_export]
macro_rules! record_frame {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident {
            $(
                $(#[$field_meta:meta])*
                $field_vis:vis $field:ident : $ty:ty
            ),* $(,)?
        }
    ) => {
        #[derive(
            $crate::rkyv::Archive,
            $crate::rkyv::Serialize,
            $crate::rkyv::Deserialize,
            Default,
            Clone,
            Debug
        )]
        #[rkyv(crate = $crate::rkyv)]
        $(#[$meta])*
        $vis struct $name {
            $(
                $(#[$field_meta])*
                $field_vis $field: $ty,
            )*
        }

        impl $crate::Interpolate for $name {
            fn interpolate(from: &Self, to: &Self, amount: f64) -> Self {
                Self {
                    $(
                        $field: $crate::Interpolate::interpolate(&from.$field, &to.$field, amount),
                    )*
                }
            }
        }
    };
}

pub mod frame;
pub mod frame_types;
pub mod lift;
pub mod routes;
pub mod runtime;
pub mod selector;
pub mod store;

pub mod prelude {
    pub use crate::{
        frame::{FrameRobot, RecordMode, Recordable},
        frame_types::{
            DifferentialCommandFrame, DifferentialPlayback, DifferentialRecording,
            DifferentialVoltagePlayback, RecordableDrivetrain,
        },
        runtime::RecordingAutonomous,
    };
}

pub use frame::Recordable;
pub use frame_types::{
    DifferentialCommandFrame, DifferentialPlayback, DifferentialRecording,
    DifferentialVoltageFrame, DifferentialVoltagePlayback, FrameType, Interpolate, RecordField,
    RecordableDrivetrain, TrackedMotionFrame, Vec2Frame, apply_differential_voltage_frame,
    apply_tracked_differential_frame,
};
