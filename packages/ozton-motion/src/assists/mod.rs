//! Stock drive assists.

use ozton_control::{drive::AntiTip, heading_corrector::HeadingCorrector, loops::Pid};
use ozton_drivetrain::Drive;
use ozton_runtime::drive_assist::DriveAssist;
pub use ozton_runtime::drive_assist::{
    AssistContext, AssistedDrivetrain, DriveAssists, MotorBalanceAssist,
};

pub struct HeadingHoldAssist(HeadingCorrector);

impl HeadingHoldAssist {
    pub fn new(gain: f64, maximum_turn: f64) -> Self {
        Self::with_pid(
            Pid::new(gain, 0.12 * gain, 0.03 * gain, Some(0.15)),
            maximum_turn,
        )
    }

    pub fn with_pid(pid: Pid, maximum_turn: f64) -> Self {
        Self(HeadingCorrector::with_pid(pid, maximum_turn))
    }
}

impl<M: Drive, T, L> DriveAssist<M, T, L> for HeadingHoldAssist {
    fn needs_orientation(&self) -> bool {
        true
    }

    fn apply(&mut self, axes: &mut [f64; 2], context: &AssistContext<'_, M, T, L>) {
        if context.wheel_velocity {
            self.0.reset();
            return;
        }
        axes[1] = self.0.turn_with_period(
            axes[0],
            axes[1],
            context.orientation.and_then(|sample| sample.heading),
            context.elapsed,
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

impl<M: Drive, T, L> DriveAssist<M, T, L> for AntiTipAssist {
    fn needs_orientation(&self) -> bool {
        true
    }

    fn apply(&mut self, axes: &mut [f64; 2], context: &AssistContext<'_, M, T, L>) {
        let tilt = context.orientation.and_then(|sample| match self.axis {
            TiltAxis::Pitch => sample.pitch,
            TiltAxis::Roll => sample.roll,
        });
        if let Some(tilt) = tilt {
            axes[0] += self.controller.correction(tilt, context.elapsed);
        } else {
            self.controller.reset();
        }
    }

    fn reset(&mut self) {
        self.controller.reset();
    }
}
