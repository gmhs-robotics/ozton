//! Common errors for robot route callbacks.

mod control_error;

pub use control_error::{ControlError, ControlResult};
pub use ozton_motion::pursuit::profiled::Follower;
