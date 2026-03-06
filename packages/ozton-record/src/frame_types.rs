use vexide::prelude::{AdiDigitalOut, Motor};

pub trait FrameType {
    type Output;
}

impl FrameType for Motor {
    type Output = f64;
}

impl FrameType for AdiDigitalOut {
    type Output = bool;
}
