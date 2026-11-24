# Your first robot

Start with a V5 Brain, a controller and at least one motor on each side of a differential drivetrain. If you already have a vexide project that uploads successfully, keep that project.

## Add Ozton

Add the following dependency to Cargo.toml. Keep the vexide version and Rust toolchain compatible with Ozton's checked-in Cargo.lock and rust-toolchain.toml.

```toml
ozton = { git = "https://github.com/gmhs-robotics/ozton", branch = "fix/motion-convergence" }
```

For a local checkout, use a path dependency pointing to packages/ozton instead. The development branch currently uses vexide 0.9.0-beta.1.

## A complete example

The code below is also checked into the repository as first_robot.rs. Change the ports, motor cartridges and direction to match your own robot. Keep the wheels off the ground for the first test.

```rust
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
```

Differential owns the left and right motor groups. Arcade::drive mixes a forward input and a turn input. Both are normalized, typically between -1 and 1; 0.25 is a quarter of the available voltage command, not a physical speed.

Runtime owns your Robot. The route macro gives the autonomous function a name in the Brain selector. Driver control repeats the controller read every control interval. The sleep is important: it lets the executor run the other tasks and notice a mode change.

The example's Forward route is intentionally timed. It demonstrates competition dispatch, not accurate distance control. Next, read [coordinates](coordinates.html) and [localization](localization.html).

## Build and try it

In the Ozton repository, use the checked-in toolchain and build the example:

```sh
cargo build -p ozton --example first_robot --release --target thumbv7a-vex-v5
```

Use your vexide upload workflow to upload the resulting firmware to the Brain. In your own project, keep the normal cargo-v5 or Nix upload command you already use. Select Forward while disabled, then enable autonomous with your field controller. Test driver control separately. If the robot turns when you command forward, correct motor polarity before doing anything else.
