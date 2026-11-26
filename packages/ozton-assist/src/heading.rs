//! Stock drive assists.

use ozton_drivetrain::Drive;
use ozton_math::{
    control::{Pid, drive::AntiTip, heading_corrector::HeadingCorrector},
    localization::observation::{ReadHeading, ReadTilt},
};

use super::{AssistContext, DriveAssist};

/// Heading hold using the configured heading capability.
///
/// ```compile_fail
/// use ozton_drivetrain::Drive;
/// use ozton_assist::{DriveAssist, HeadingHoldAssist};
/// fn require<D: Drive, A: DriveAssist<D>>() {}
/// fn missing_heading<M: Drive>() {
///     require::<M, HeadingHoldAssist>();
/// }
/// ```
pub struct HeadingHoldAssist(HeadingCorrector);

impl HeadingHoldAssist {
    pub const fn new(gain: f64, maximum_turn: f64) -> Self {
        Self::with_pid(
            Pid::new(gain, 0.12 * gain, 0.03 * gain, Some(0.15)),
            maximum_turn,
        )
    }

    pub const fn with_pid(pid: Pid, maximum_turn: f64) -> Self {
        Self(HeadingCorrector::with_pid(pid, maximum_turn))
    }
}

impl<D: Drive + ReadHeading + ?Sized> DriveAssist<D> for HeadingHoldAssist {
    fn apply(&mut self, axes: &mut [f64; 2], context: &AssistContext<'_, D>) {
        if context.wheel_velocity {
            self.0.reset();
            return;
        }
        axes[1] = self.0.turn_with_period(
            axes[0],
            axes[1],
            context.drive.read_heading().map(|heading| -heading),
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

/// Anti-tip requires a source measuring both pitch and roll.
///
/// ```compile_fail
/// use ozton_drivetrain::Drive;
/// use ozton_assist::{AntiTipAssist, DriveAssist};
/// fn require<D: Drive, A: DriveAssist<D>>() {}
/// fn missing_tilt<M: Drive>() {
///     require::<M, AntiTipAssist>();
/// }
/// ```
pub struct AntiTipAssist {
    axis: TiltAxis,
    controller: AntiTip,
}

impl AntiTipAssist {
    pub const fn new(axis: TiltAxis, controller: AntiTip) -> Self {
        Self { axis, controller }
    }
}

impl<D: Drive + ReadTilt + ?Sized> DriveAssist<D> for AntiTipAssist {
    fn apply(&mut self, axes: &mut [f64; 2], context: &AssistContext<'_, D>) {
        let tilt = context
            .drive
            .read_tilt()
            .map(|[pitch, roll]| match self.axis {
                TiltAxis::Pitch => pitch,
                TiltAxis::Roll => roll,
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
