use ozton::record::frame::Recordable;
use vexide::{prelude::*, smart::PortError};

// This macro creates another struct named RobotFrame which maps all smart devices which can be
// replayed into primitives to be saved. IE: motors go to f64's of velocity, AdiDevices go to
// booleans.
#[derive(ozton::derive::RobotFrame)]
struct Robot {
    #[frame(skip)]
    primary_controller: Controller,

    left: [Motor; 3],
    right: [Motor; 3],
}

#[async_trait::async_trait(?Send)]
impl Recordable for Robot {
    type Frame = RobotFrame;
    const UPDATE_INTERVAL: std::time::Duration = Controller::UPDATE_INTERVAL;

    async fn get_new_frame(&self) -> Self::Frame {
        let controller_state = self.primary_controller.state().unwrap_or_default();

        let t = controller_state.right_stick.y();
        let r = controller_state.left_stick.x();

        let left_volts = (t + r) * Motor::V5_MAX_VOLTAGE;
        let right_volts = (t - r) * MAX_Motor::V5_MAX_VOLTAGEWHEEL;

        Self::Frame {
            left: left_volts,
            right: right_volts,
        }
    }

    // Data is directly passed from get -> transform in drive
    // You can see the split here!
    //
    // Ordinarily, without this library for recording, you could delete the head of this
    // function and the tail of the previous function and set that as your drive method and it
    // would work as intended. The reason we split it is to save frames to an SD card for
    // later playback

    async fn transform_to_frame(&mut self, frame: &Self::Frame) -> Result<(), PortError> {
        for motor in &mut self.left {
            let _ = motor.set_voltage(frame.left);
        }

        for motor in &mut self.right {
            let _ = motor.set_voltage(frame.right);
        }

        Ok(())
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let primary_controller = peripherals.primary_controller;

    let left = [
        Motor::new(peripherals.port_1, Gearset::Green, Direction::Reverse),
        Motor::new(peripherals.port_2, Gearset::Green, Direction::Reverse),
    ];
    let right = [
        Motor::new(peripherals.port_3, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_4, Gearset::Green, Direction::Forward),
    ];

    let display = peripherals.display;

    let robot = Robot {
        primary_controller,

        left,
        right,
    };

    // You will need to build two programs, a recorder and a playback.
    // This can be easily accomplished by a feature flag + build script
    //
    // cargo v5 upload --name banana-leclerc -d "Main program" -s 1 -i vex-coding-studio --release
    // cargo v5 upload --name recorder -d "Auton recorder" -s 2 -i code-file --release --features record
    if cfg!(feature = "record") {
        ozton::record::runtime::RecordingAutonomous::compete(robot, display).await;
    } else {
        ozton::record::runtime::PlaybackAutonomous::compete(robot, display).await;
    };
}
