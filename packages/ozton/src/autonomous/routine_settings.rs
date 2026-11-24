//! Default limits and motion tuning for programmed routes.
use std::time::Duration;

use ozton_math::uom::{
    ConstZero,
    si::f64::{Acceleration, Length, Velocity},
};
pub const AUTON_STAGE_TIMEOUT: Duration = ozton_motion::MotionConfig::DEFAULT.timing.stage_timeout;
pub const AUTON_SPEED: Velocity = Velocity {
    value: 0.40,
    ..Velocity::ZERO
};

pub const DSR_TIMEOUT: Duration = Duration::from_secs(1);
pub const PATH_LOOKAHEAD: Length = ozton_math::qty!(0.16 m);
pub const PATH_ACCELERATION: Acceleration = Acceleration {
    value: 0.8,
    ..Acceleration::ZERO
};
pub const PATH_DECELERATION: Acceleration = Acceleration {
    value: 1.0,
    ..Acceleration::ZERO
};
pub const PATH_LATERAL_ACCELERATION: Acceleration = Acceleration {
    value: 0.8,
    ..Acceleration::ZERO
};
pub const ALIGN_SPEED: Velocity = Velocity {
    value: 0.15,
    ..Velocity::ZERO
};
