use std::time::Duration;

use ozton::{
    derive::RecordedRobot,
    drivetrain::model::Differential,
    prelude::{NoTracking, RecordableDrivetrain},
    record::{DifferentialVoltageFrame, RecordableRobot},
};

#[allow(dead_code)]
#[derive(RecordedRobot)]
struct Robot {
    drivetrain: RecordableDrivetrain<Differential, NoTracking>,
}

#[ozton::record::async_trait(?Send)]
impl RecordableRobot for Robot {
    const UPDATE_INTERVAL: Duration = Duration::from_millis(10);

    async fn get_new_frame(&self) -> Self::Frame {
        Self::Frame {
            drivetrain: DifferentialVoltageFrame::tank(0.0, 0.0),
        }
    }
}

#[test]
fn recordable_drivetrain_is_publicly_usable() {
    let _frame = RobotFrame::default();
}
