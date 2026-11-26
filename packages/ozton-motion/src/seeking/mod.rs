//! Shared execution for independently typed motion controllers.

mod controllers;
mod pose;
pub use controllers::*;
pub use pose::{
    MotionChain, MotionSegment, PathActions, PoseMotionError, RunMotion, TravelDirection,
    WallContact,
};

/// Private execution configuration, not a public controller.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Engine {
    config: crate::MotionConfig,
    travel: Option<TravelDirection>,
}
