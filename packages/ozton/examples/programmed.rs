//! One mixed-gear chassis, one intake, and one embedded PATH.JERRYIO route.

use std::time::Duration;

use ozton::{
    drivetrain::model::GearRatio,
    math::{
        geometry::{INCH, Pose},
        localization::range::{Mount, Surface},
    },
    motion::auton::{
        AlignTarget, Auton, DifferentialConfig, Mechanisms, Robot, Route, RouteResult,
        TrackingSetup, compete_routes,
    },
    tracking::localizer::RangeSensorConfig,
};
use vexide::{
    prelude::*,
    smart::{PortError, motor::BrakeMode},
};

const START: Pose = Pose::new(0.0, 0.0, 0.0);
const GEARED: GearRatio = GearRatio::from_teeth(36, 48);

struct Intake {
    motor: Motor,
    on: bool,
}

impl Mechanisms for Intake {
    fn update(&mut self, _: Duration) -> Result<(), PortError> {
        self.motor.set_voltage(if self.on { 5.0 } else { 0.0 })
    }

    fn stop(&mut self) -> Result<(), PortError> {
        self.on = false;
        self.motor.brake(BrakeMode::Brake)
    }
}

async fn straight(mut auton: Auton<'_, Intake>) -> RouteResult {
    auton.mechanisms().on = true;
    auton.follow(Duration::from_secs(5)).await?;
    auton.mechanisms().on = false;
    let front = auton.range("front")?;
    // This alignment is optional. The route can continue after a clean timeout.
    auton
        .try_align(front, AlignTarget::front(1.10, 0.0), Duration::from_secs(2))
        .await?;
    Ok(())
}

#[vexide::main]
async fn main(p: Peripherals) {
    let mut config = DifferentialConfig::new(2.25 * INCH, 0.30, 0.50);
    config.journal_capacity = 2048;
    let side = |direct, geared, direction| {
        [
            (Motor::new_exp(direct, direction), GearRatio::DIRECT),
            (Motor::new(geared, Gearset::Blue, direction), GEARED),
        ]
    };
    let left = side(p.port_1, p.port_2, Direction::Reverse);
    let right = side(p.port_3, p.port_4, Direction::Forward);
    let tracking = TrackingSetup::default()
        .with_imu(InertialSensor::new(p.port_7))
        .with_map([Surface::new(1.8, -1.8, 1.8, 1.8)])
        .with_range(
            "front",
            DistanceSensor::new(p.port_8),
            RangeSensorConfig::new(Mount::new(0.15, 0.0, 0.0)),
        );
    let drive = config
        .build(left, right, tracking)
        .await
        .expect("valid chassis");
    let intake = Intake {
        motor: Motor::new(p.port_5, Gearset::Green, Direction::Forward),
        on: false,
    };
    let robot = Robot::new(drive, intake)
        .with_driver(|intake, state| intake.on = state.button_r1.is_pressed())
        .with_log_file("auton.csv");

    let route = Route::jerryio(
        "Straight",
        START,
        include_bytes!("paths/straight.txt"),
        straight,
    );
    compete_routes(robot, p.display, p.primary_controller, [route])
        .await
        .expect("valid routes");
}
