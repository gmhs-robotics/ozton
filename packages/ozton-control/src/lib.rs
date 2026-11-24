//! Control API forwarded to the hardware-independent math crate.
pub use ozton_math::control::{axis, controller, drive, heading_corrector, lift, limits, protection};
pub mod loops {
    pub use ozton_math::control::{
        ArmFeedforward, ArmFeedforwardSetpoint, BangBang, ElevatorFeedforward,
        ElevatorFeedforwardSetpoint, Feedback, Feedforward, MotorFeedforward,
        MotorFeedforwardSetpoint, Pid, TakeBackHalf,
    };
}
