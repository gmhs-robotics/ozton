//! Framing for the recording system, includes archiving.

use vexide::smart::{PortError, motor::Motor};

pub trait FrameType {
    type Output;

    fn set_output(&mut self, out: Self::Output) -> Result<(), PortError>;
}

impl FrameType for Motor {
    type Output = f64;

    fn set_output(&mut self, out: Self::Output) -> Result<(), PortError> {
        self.set_voltage(out)
    }
}
