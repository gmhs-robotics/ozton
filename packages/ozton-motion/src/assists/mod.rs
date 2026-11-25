//! Stock drive assists.

pub use ozton_drivetrain::assist::MotorBalanceAssist;
use ozton_drivetrain::{
    Drive,
    assist::{AssistContext, DriveAssist},
};
use ozton_math::{
    control::{Pid, drive::AntiTip, heading_corrector::HeadingCorrector},
    localization::observation::{ReadHeading, ReadTilt, SourceAt},
};

/// Heading hold using the heading-capable source at tuple index I.
pub struct HeadingHoldAssist<const I: usize>(HeadingCorrector);

impl<const I: usize> HeadingHoldAssist<I> {
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

impl<M: Drive, T: SourceAt<I>, L, const I: usize> DriveAssist<M, T, L> for HeadingHoldAssist<I>
where
    T::Source: ReadHeading,
{
    fn apply(&mut self, axes: &mut [f64; 2], context: &AssistContext<'_, M, T, L>) {
        if context.wheel_velocity {
            self.0.reset();
            return;
        }
        axes[1] = self.0.turn_with_period(
            axes[0],
            axes[1],
            context.tracking.source_at().read_heading().map(|heading| -heading),
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
/// use ozton_drivetrain::{Drive, assist::DriveAssist};
/// use ozton_motion::assists::AntiTipAssist;
/// fn require<M: Drive, A: DriveAssist<M, (), ()>>() {}
/// fn missing_tilt<M: Drive>() {
///     require::<M, AntiTipAssist<0>>();
/// }
/// ```
pub struct AntiTipAssist<const I: usize> {
    axis: TiltAxis,
    controller: AntiTip,
}

impl<const I: usize> AntiTipAssist<I> {
    pub const fn new(axis: TiltAxis, controller: AntiTip) -> Self {
        Self { axis, controller }
    }
}

impl<M: Drive, T: SourceAt<I>, L, const I: usize> DriveAssist<M, T, L> for AntiTipAssist<I>
where
    T::Source: ReadTilt,
{
    fn apply(&mut self, axes: &mut [f64; 2], context: &AssistContext<'_, M, T, L>) {
        let tilt = context.tracking.source_at().read_tilt().map(|[pitch, roll]| match self.axis {
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
