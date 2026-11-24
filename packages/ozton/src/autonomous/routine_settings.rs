//! Default limits and motion tuning for programmed routes.
use std::time::Duration;

use ozton_math::uom::{
    ConstZero,
    si::f64::{Acceleration, Length, Velocity},
};
use ozton_motion::seeking::pose::DEFAULT_ROUTE_BUDGET as AUTON_TOTAL_TIMEOUT;
pub use ozton_motion::seeking::pose::DEFAULT_STAGE_TIMEOUT as AUTON_STAGE_TIMEOUT;
const AUTON_SETTLE_TIME: Duration = Duration::from_millis(150);
pub const AUTON_SPEED: Velocity = Velocity {
    value: 0.40,
    ..Velocity::ZERO
};

pub const DSR_TIMEOUT: Duration = Duration::from_secs(1);
pub const PATH_LOOKAHEAD: Length = ozton_math::length!(0.16, m);
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

/// Per-route deadlines. Both match and skills routes keep one session for all
/// shared helpers and loop iterations. Field disable always cancels either mode.
#[derive(Clone, Copy, Debug)]
pub struct SessionLimits {
    pub total: Duration,
    pub stage: Duration,
}
impl SessionLimits {
    pub const MATCH: Self = Self {
        total: AUTON_TOTAL_TIMEOUT,
        stage: AUTON_STAGE_TIMEOUT,
    };
    pub const SKILLS: Self = Self {
        total: Duration::from_secs(60),
        stage: AUTON_STAGE_TIMEOUT,
    };
    pub const fn valid(self) -> bool {
        self.stage.as_nanos() > AUTON_SETTLE_TIME.as_nanos()
            && self.stage.as_nanos() <= self.total.as_nanos()
            && self.total.as_nanos() <= Self::SKILLS.total.as_nanos()
    }
}
