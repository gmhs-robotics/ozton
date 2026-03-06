use std::{cell::RefCell, time::Duration};

use ozton_record::{
    prelude::Recordable,
    rkyv::{Archive, Deserialize, Serialize},
};
use vexide::smart::PortError;

#[derive(Archive, Serialize, Deserialize, Default, Clone, Debug)]
#[rkyv(crate = ::ozton_record::rkyv)]
struct DummyFrame {
    value: u8,
}

struct NonSendRobot {
    value: RefCell<u8>,
}

#[async_trait::async_trait(?Send)]
impl Recordable for NonSendRobot {
    type Frame = DummyFrame;
    const UPDATE_INTERVAL: Duration = Duration::from_millis(20);

    async fn transform_to_frame(&mut self, frame: &Self::Frame) -> Result<(), PortError> {
        *self.value.get_mut() = frame.value;
        Ok(())
    }

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
