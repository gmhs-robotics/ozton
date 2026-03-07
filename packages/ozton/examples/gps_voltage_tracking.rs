use std::time::Duration;

use ozton::{
    derive::RecordedRobot,
    drivetrain::model::Differential,
    prelude::{Drivetrain, RecordableDrivetrain},
    record::{
        self,
        DifferentialVoltageFrame,
        frame::Recordable,
        runtime::{PlaybackAutonomous, RecordingAutonomous},
    },
    tracking::GpsTracking,
};
use vexide::{math::Point2, prelude::*};

#[derive(RecordedRobot)]
struct Robot {
    #[record(skip)]
    controller: Controller,

    drivetrain: RecordableDrivetrain<Differential, GpsTracking>,
}

#[record::async_trait(?Send)]
impl Recordable for Robot {
    const UPDATE_INTERVAL: Duration = Controller::UPDATE_INTERVAL;

    async fn get_new_frame(&self) -> Self::Frame {
        let state = self.controller.state().unwrap_or_default();

        Self::Frame {
            drivetrain: DifferentialVoltageFrame::arcade(
                state.left_stick.y(),
                state.right_stick.x(),
            ),
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let display = peripherals.display;

    let gps = GpsSensor::new(
        peripherals.port_7,
        Point2 { x: 0.0, y: 0.0 },
        Point2 { x: 0.0, y: 0.0 },
        0.0,
    );

    let robot = Robot {
        controller: peripherals.primary_controller,
        drivetrain: RecordableDrivetrain::new(Drivetrain::new(
            Differential::new(
                [
                    Motor::new(peripherals.port_1, Gearset::Green, Direction::Reverse),
                    Motor::new(peripherals.port_2, Gearset::Green, Direction::Reverse),
                ],
                [
                    Motor::new(peripherals.port_3, Gearset::Green, Direction::Forward),
                    Motor::new(peripherals.port_4, Gearset::Green, Direction::Forward),
                ],
            ),
            GpsTracking::new(gps),
        )),
    };

    if cfg!(feature = "record") {
        RecordingAutonomous::compete(robot, display).await;
    } else {
        PlaybackAutonomous::compete(robot, display).await;
    }
}
