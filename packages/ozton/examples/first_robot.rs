//! A first V5 robot: two motors, driver control and a timed autonomous.
use std::time::Duration;

use ozton::{
    drivetrain::model::Differential,
    motion::Arcade,
    route,
    runtime::{OztonCompete, Runtime, competition::{RobotRoute, Ui}},
};
use vexide::{prelude::*, smart::PortError, time::sleep};

struct Robot {
    drive: Differential,
    controller: Controller,
}

impl Robot {
    async fn forward(&mut self) -> Result<(), PortError> {
        Arcade::drive(&mut self.drive, 0.25, 0.0)?;
        sleep(Duration::from_millis(500)).await;
        self.stop()
    }
}

impl OztonCompete for Robot {
    type Error = PortError;
    const ROUTES: &'static [RobotRoute<Self, Self::Error>] =
        &[route!(for Self, "Forward", Self::forward)];

    fn stop(&mut self) -> Result<(), Self::Error> {
        Arcade::drive(&mut self.drive, 0.0, 0.0)
    }

    async fn driver(&mut self) -> Result<(), Self::Error> {
        loop {
            if let Ok(input) = self.controller.state() {
                Arcade::drive(&mut self.drive, input.left_stick.y(), input.right_stick.x())?;
            } else {
                self.stop()?;
            }
            sleep(Self::CONTROL_PERIOD).await;
        }
    }
}

#[vexide::main]
async fn main(p: Peripherals) {
    let robot = Robot {
        drive: Differential::new(
            [Motor::new(p.port_1, Gearset::Green, Direction::Reverse)],
            [Motor::new(p.port_2, Gearset::Green, Direction::Forward)],
        ),
        controller: p.primary_controller,
    };
    Runtime::new(robot).with_ui(Ui::Brain(p.display)).compete().await;
}
