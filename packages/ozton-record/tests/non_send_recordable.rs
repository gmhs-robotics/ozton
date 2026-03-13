use std::{cell::RefCell, time::Duration};

use ozton_record::{
    Interpolate, RecordableRobot,
    frame::{FrameRobot, RecordMode},
    rkyv::{Archive, Deserialize, Serialize},
};
use vexide::smart::PortError;

#[derive(Archive, Serialize, Deserialize, Default, Clone, Debug)]
#[rkyv(crate = ::ozton_record::rkyv)]
struct DummyFrame {
    value: u8,
}

impl Interpolate for DummyFrame {
    fn interpolate(from: &Self, to: &Self, amount: f64) -> Self {
        Self {
            value: Interpolate::interpolate(&from.value, &to.value, amount),
        }
    }
}

struct NonSendRobot {
    value: RefCell<u8>,
}

#[async_trait::async_trait(?Send)]
impl FrameRobot for NonSendRobot {
    type Frame = DummyFrame;

    async fn finalize_frame(&self, frame: &Self::Frame) -> Self::Frame {
        frame.clone()
    }

    async fn apply_frame(
        &mut self,
        frame: &Self::Frame,
        _mode: RecordMode,
    ) -> Result<(), PortError> {
        *self.value.get_mut() = frame.value;
        Ok(())
    }
}

#[async_trait::async_trait(?Send)]
impl RecordableRobot for NonSendRobot {
    const UPDATE_INTERVAL: Duration = Duration::from_millis(20);

    async fn get_new_frame(&self) -> Self::Frame {
        DummyFrame {
            value: *self.value.borrow(),
        }
    }
}

#[test]
fn non_send_recordable_impl_compiles() {
    let _robot = NonSendRobot {
        value: RefCell::new(0),
    };
}
