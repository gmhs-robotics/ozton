#![feature(trait_alias)]

pub use async_trait::async_trait;
pub use rkyv;
pub use vexide::smart::PortError;

#[macro_export]
macro_rules! log {
    ($($arg:tt)*) => {
        #[cfg(debug_assertions)]
        {
            ::std::println!("[ozton] {}", ::core::format_args!($($arg)*));
        }
    };
}

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
pub mod routes;
pub mod runtime;
pub mod selector;

pub mod prelude {
    pub use crate::{
        frame::{FrameRobot, RecordMode, Recordable as RecordableRobot},
        frame_types::{
            DifferentialCommandFrame, DifferentialPlayback, DifferentialRecording,
            DifferentialVoltagePlayback, Recordable, RecordableDrivetrain, RecordedDifferential,
        },
        runtime::{PlaybackAutonomous, RecordingAutonomous},
    };
}

pub use frame::Recordable as RecordableRobot;
pub use frame_types::{
    DifferentialCommandFrame, DifferentialPlayback, DifferentialRecording,
    DifferentialVoltageFrame, DifferentialVoltagePlayback, FrameType, Interpolate, RecordField,
    Recordable, RecordableDrivetrain, RecordedDifferential, TrackedMotionFrame, Vec2Frame,
    apply_differential_voltage_frame, apply_tracked_differential_frame,
};
