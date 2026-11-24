//! Default limits and motion tuning for programmed routes.
use std::time::Duration;

use ozton_math::uom::{
    ConstZero,
    si::f64::{Angle, AngularVelocity, Length, Velocity},
};

// Autonomous tuning
pub const AUTON_STAGE_TIMEOUT: Duration = Duration::from_secs(8);
pub const AUTON_TOTAL_TIMEOUT: Duration = Duration::from_secs(15);
pub const AUTON_POSITION_TOLERANCE: Length = Length {
    value: 0.025,
    ..Length::ZERO
};
pub const AUTON_HEADING_TOLERANCE: Angle = Angle {
    value: 0.06,
    ..Angle::ZERO
};
pub const AUTON_SPEED_TOLERANCE: Velocity = Velocity {
    value: 0.04,
    ..Velocity::ZERO
};
pub const AUTON_ANGULAR_SPEED_TOLERANCE: AngularVelocity = AngularVelocity {
    value: 0.12,
    ..AngularVelocity::ZERO
};
pub const AUTON_SETTLE_TIME: Duration = Duration::from_millis(150);

pub const AUTON_SPEED: Velocity = Velocity {
    value: 0.40,
    ..Velocity::ZERO
};
