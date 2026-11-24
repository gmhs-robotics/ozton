//! Motion control loops and their unit-bearing adapters.

pub use ozton_math::control::{
    ArmFeedforward, ArmFeedforwardSetpoint, BangBang, ElevatorFeedforward,
    ElevatorFeedforwardSetpoint, Feedback, Feedforward, MotorFeedforward, MotorFeedforwardSetpoint,
    Pid, TakeBackHalf,
};
