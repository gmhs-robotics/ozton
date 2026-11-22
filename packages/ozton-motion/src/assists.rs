//! Stock drive assists and timed assisted motion.

use std::time::Duration;

use ozton_control::{drive::AntiTip, heading_corrector::HeadingCorrector};
use ozton_drivetrain::{assist::DriveAssist, model::DrivetrainModel};
use ozton_tracking::sources::vex::Orientation;

mod timed;

pub use ozton_drivetrain::assist::{AssistChain, AssistedDrivetrain, MotorBalanceAssist};
pub use timed::AssistedMotion;

pub struct HeadingHoldAssist(HeadingCorrector);

impl HeadingHoldAssist {
    pub fn new(gain: f64, maximum_turn: f64) -> Self {
        Self(HeadingCorrector::new(gain, maximum_turn))
    }
}

impl<M: DrivetrainModel> DriveAssist<M> for HeadingHoldAssist {
    fn needs_orientation(&self) -> bool {
        true
    }

    fn apply(&mut self, axes: &mut [f64; 2], orientation: Option<Orientation>, _: Duration) {
        axes[1] = self.0.turn(
            axes[0],
            axes[1],
            orientation.and_then(|sample| sample.heading),
        );
    }

    fn reset(&mut self) {
        self.0.reset();
    }
}

/// IMU axis used for fore-aft tilt on the installed robot.
#[derive(Clone, Copy)]
pub enum TiltAxis {
    Pitch,
    Roll,
}

pub struct AntiTipAssist {
    axis: TiltAxis,
    controller: AntiTip,
}

impl AntiTipAssist {
    pub fn new(axis: TiltAxis, controller: AntiTip) -> Self {
        Self { axis, controller }
    }
}

impl<M: DrivetrainModel> DriveAssist<M> for AntiTipAssist {
    fn needs_orientation(&self) -> bool {
        true
    }

    fn apply(&mut self, axes: &mut [f64; 2], orientation: Option<Orientation>, dt: Duration) {
        let tilt = orientation.and_then(|sample| match self.axis {
            TiltAxis::Pitch => sample.pitch,
            TiltAxis::Roll => sample.roll,
        });
        if let Some(tilt) = tilt {
            axes[0] += self.controller.correction(tilt, dt);
        } else {
            self.controller.reset();
        }
    }

    fn reset(&mut self) {
        self.controller.reset();
    }
}
