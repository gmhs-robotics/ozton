use std::time::Duration;

use ozton::{
    derive::RecordedRobot,
    record::{FrameType, frame::Recordable},
};

struct AnalogField;

#[ozton::record::async_trait(?Send)]
impl ozton::record::RecordField for AnalogField {
    type Output = f64;

    async fn apply_frame_value(
        &mut self,
        _frame: &Self::Output,
        _mode: ozton::record::frame::RecordMode,
    ) -> Result<(), ozton::record::PortError> {
        Ok(())
    }
}

#[derive(RecordedRobot)]
struct Robot {
    analog: AnalogField,
}

#[ozton::record::async_trait(?Send)]
impl Recordable for Robot {
    const UPDATE_INTERVAL: Duration = Duration::from_millis(10);

    async fn get_new_frame(&self) -> Self::Frame {
        Self::Frame { analog: 42.0 }
    }
}

#[test]
fn derive_works_via_ozton() {
    let frame = RobotFrame { analog: 42.0 };
    let _: <AnalogField as FrameType>::Output = frame.analog;

    let _robot = Robot {
        analog: AnalogField,
    };
}
