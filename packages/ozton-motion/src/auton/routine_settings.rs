//! Default limits and motion tuning for programmed routes.
use std::time::Duration;

use ozton_math::uom::{
    ConstZero,
    si::f64::{Acceleration, Angle, AngularVelocity, Length, Velocity},
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
pub const DSR_TIMEOUT: Duration = Duration::from_secs(1);
pub const PATH_LOOKAHEAD: Length = Length {
    value: 0.16,
    ..Length::ZERO
};
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
