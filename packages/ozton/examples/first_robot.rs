//! A first V5 robot: two motors, driver control and a timed autonomous.
use std::time::Duration;

use ozton::{
    drivetrain::{Drive, model::Differential},
    route,
    runtime::{OztonCompete, competition::RobotRoute},
};
use vexide::{prelude::*, smart::PortError, time::sleep};

struct Robot {
    drive: Differential,
    controller: Controller,
}

impl Robot {
    async fn forward(&mut self) -> Result<(), PortError> {
        self.drive.arcade(0.25, 0.0)?;
        sleep(Duration::from_millis(500)).await;
        self.stop()
    }
}

impl OztonCompete for Robot {
    type Error = PortError;
    const ROUTES: &'static [RobotRoute<Self>] = &[route!("Forward", Self::forward)];

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.drive.arcade(0.0, 0.0)
    }

    async fn driver(&mut self) -> Result<(), Self::Error> {
        loop {
            if let Ok(input) = self.controller.state() {
                self.drive
                    .arcade(input.left_stick.y(), input.right_stick.x())?;
            } else {
                self.stop()?;
            }
            sleep(Self::CONTROL_PERIOD).await;
        }
    }
}

#[vexide::main]
async fn main(p: Peripherals) -> std::process::ExitCode {
    ozton::run(async move {
        let robot = Robot {
            drive: Differential::new(
                [Motor::new(p.port_1, Gearset::Green, Direction::Reverse)],
                [Motor::new(p.port_2, Gearset::Green, Direction::Forward)],
            ),
            controller: p.primary_controller,
        };
        Ok::<_, PortError>(robot.with_brain(p.display))
    })
    .await
}
