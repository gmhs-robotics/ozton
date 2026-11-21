//! Control theory primitives.
//!
//! This module provides basic building blocks and implementations for controlling
//! systems. These "systems" could be drivetrains, an arm or lift, or any other
//! mechanism that requires precise motion control.

pub mod axis;
pub mod drive;
pub mod lift;
pub mod loops;
pub mod low_score;

mod tolerances;
pub use tolerances::{Settling, Tolerances};

pub mod heading_hold;
pub mod protection;

pub mod limits;
