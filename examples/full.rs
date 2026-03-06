use ozton::prelude::*;
use vexide::prelude::*;

#[oz(robot)]
struct Robot {
    controller: Controller,

    drivetrain: Drivetrain,
}

#[vexide::main]
async fn main(peripherals: Peripherals) {}
