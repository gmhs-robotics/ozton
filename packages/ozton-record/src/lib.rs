#![feature(trait_alias)]

pub use rkyv;

pub mod frame;
pub mod frame_types;
pub mod routes;
pub mod runtime;
pub mod selector;

pub mod prelude {
    pub use crate::{
        frame::Recordable,
        runtime::{PlaybackAutonomous, RecordingAutonomous},
    };
}

pub use frame_types::FrameType;
