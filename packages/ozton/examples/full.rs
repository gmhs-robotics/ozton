use std::time::Duration;

use ozton::{
    derive::RecordedRobot,
    drivetrain::model::Differential,
    prelude::{Drivetrain, NoTracking, RecordableDrivetrain},
    record::{
        self,
        frame::Recordable,
        runtime::{PlaybackAutonomous, RecordingAutonomous},
    },
};
use vexide::prelude::*;

#[derive(RecordedRobot)]
struct Robot {
    #[record(skip)]
    controller: Controller,

    drivetrain: RecordableDrivetrain<Differential, NoTracking>,
    intake: Motor,
}

#[record::async_trait(?Send)]
impl Recordable for Robot {
    const UPDATE_INTERVAL: Duration = Controller::UPDATE_INTERVAL;

    async fn get_new_frame(&self) -> Self::Frame {
        let state = self.controller.state().unwrap_or_default();
        let intake = match (state.button_r1.is_pressed(), state.button_r2.is_pressed()) {
            (true, false) => 1.0,
            (false, true) => -1.0,
            _ => 0.0,
        };

        Self::Frame {
            drivetrain: ozton::record::DifferentialVoltageFrame::arcade(
                state.left_stick.y(),
                state.right_stick.x(),
            ),
            intake,
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let display = peripherals.display;

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
            NoTracking,
        )),
        intake: Motor::new(peripherals.port_5, Gearset::Green, Direction::Forward),
    };

    if cfg!(feature = "record") {
        RecordingAutonomous::compete(robot, display).await;
    } else {
        PlaybackAutonomous::compete(robot, display).await;
    }
}
