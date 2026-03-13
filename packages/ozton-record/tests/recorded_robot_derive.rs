use std::{cell::RefCell, time::Duration};

use futures::executor::block_on;
use ozton_derive::RecordedRobot;
use ozton_record::{
    FrameType, Interpolate, PortError, RecordField, Recordable, RecordableRobot,
    frame::{FrameRobot, RecordMode},
};

struct AnalogField(RefCell<f64>);
struct DigitalField(RefCell<bool>);

#[async_trait::async_trait(?Send)]
impl RecordField for AnalogField {
    type Output = f64;

    async fn finalize_frame_value(&self, frame: &Self::Output) -> Self::Output {
        frame + 1.0
    }

    async fn apply_frame_value(
        &mut self,
        frame: &Self::Output,
        _mode: RecordMode,
    ) -> Result<(), PortError> {
        *self.0.borrow_mut() = *frame;
        Ok(())
    }
}

#[async_trait::async_trait(?Send)]
impl RecordField for DigitalField {
    type Output = bool;

    async fn apply_frame_value(
        &mut self,
        frame: &Self::Output,
        _mode: RecordMode,
    ) -> Result<(), PortError> {
        *self.0.borrow_mut() = *frame;
        Ok(())
    }
}

#[derive(RecordedRobot)]
struct TestRobot {
    analog: Recordable<AnalogField>,
    digital: Recordable<DigitalField>,
    #[record(skip)]
    skipped: u8,
}

#[async_trait::async_trait(?Send)]
impl RecordableRobot for TestRobot {
    const UPDATE_INTERVAL: Duration = Duration::from_millis(10);

    async fn get_new_frame(&self) -> Self::Frame {
        Self::Frame {
            analog: 3.0,
            digital: true,
        }
    }
}

#[test]
fn derive_generates_frametype_outputs() {
    let frame = TestRobotFrame {
        analog: 1.5,
        digital: false,
    };

    let TestRobotFrame { analog, digital } = frame;
    let _: <Recordable<AnalogField> as FrameType>::Output = analog;
    let _: <Recordable<DigitalField> as FrameType>::Output = digital;
}

#[test]
fn derive_applies_and_finalizes_fields() {
    let mut robot = TestRobot {
        analog: Recordable::new(AnalogField(RefCell::new(0.0))),
        digital: Recordable::new(DigitalField(RefCell::new(false))),
        skipped: 7,
    };

    let frame = block_on(robot.get_new_frame());
    let finalized = block_on(robot.finalize_frame(&frame));

    assert!((finalized.analog - 4.0).abs() < 1e-9);
    assert!(finalized.digital);

    block_on(robot.apply_frame(
        &TestRobotFrame {
            analog: 2.5,
            digital: true,
        },
        RecordMode::Playback,
    ))
    .unwrap();

    assert!((*robot.analog.0.0.borrow() - 2.5).abs() < 1e-9);
    assert!(*robot.digital.0.0.borrow());
    assert_eq!(robot.skipped, 7);
}

#[test]
fn generated_frame_interpolates() {
    let from = TestRobotFrame {
        analog: 0.0,
        digital: false,
    };
    let to = TestRobotFrame {
        analog: 8.0,
        digital: true,
    };

    let mid = TestRobotFrame::interpolate(&from, &to, 0.25);
    assert!((mid.analog - 2.0).abs() < 1e-9);
    assert!(!mid.digital);
}
