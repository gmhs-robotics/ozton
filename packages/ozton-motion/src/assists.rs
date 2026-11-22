//! Opt-in assists for arcade commands in driver control or autonomous routes.

use std::time::Duration;

use ozton_control::{drive::AntiTip, heading_corrector::HeadingCorrector};
use ozton_drivetrain::{
    Drivetrain,
    model::{ArcadeInput, Balancing, DrivetrainModel, Tank, arcade_mix},
};
use ozton_tracking::sampling::SensorTracking;

mod timed;

const MAX_CONTROL_GAP: Duration = Duration::from_millis(100);

/// IMU axis used for fore-aft tilt on the installed robot.
#[derive(Clone, Copy)]
pub enum TiltAxis {
    Pitch,
    Roll,
}

/// Composes heading hold and anti-tip. Keep a separate instance for each
/// independently controlled segment or reset it before reusing it.
#[derive(Default)]
pub struct DriveAssists {
    heading: Option<HeadingCorrector>,
    anti_tip: Option<(TiltAxis, AntiTip)>,
}

impl DriveAssists {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_heading_hold(mut self, gain: f64, maximum_turn: f64) -> Self {
        self.heading = Some(HeadingCorrector::new(gain, maximum_turn));
        self
    }

    pub fn with_anti_tip(mut self, axis: TiltAxis, controller: AntiTip) -> Self {
        self.anti_tip = Some((axis, controller));
        self
    }

    pub fn reset(&mut self) {
        if let Some(heading) = &mut self.heading {
            heading.reset();
        }
        if let Some((_, anti_tip)) = &mut self.anti_tip {
            anti_tip.reset();
        }
    }

    /// Apply one assisted command to a tank or arcade drivetrain.
    pub fn drive<M: Tank, L>(
        &mut self,
        drive: &mut Drivetrain<M, SensorTracking, L>,
        input: ArcadeInput,
        dt: Duration,
    ) -> Result<(), M::Error> {
        let [left, right] = self.wheel_commands(drive, input, dt);
        drive.drive_tank(left, right)
    }

    /// Apply one assisted command with the model's motor-loss policy.
    pub fn drive_balanced<M: Balancing<Command = [f64; 2]>, L>(
        &mut self,
        drive: &mut Drivetrain<M, SensorTracking, L>,
        input: ArcadeInput,
        dt: Duration,
    ) -> Result<(), M::Error> {
        let wheels = self.wheel_commands(drive, input, dt);
        drive.drive_balanced(wheels)
    }

    fn wheel_commands<M: DrivetrainModel, L>(
        &mut self,
        drive: &mut Drivetrain<M, SensorTracking, L>,
        input: ArcadeInput,
        dt: Duration,
    ) -> [f64; 2] {
        if dt.is_zero() || dt > MAX_CONTROL_GAP {
            self.reset();
        }
        let (forward, steer) = input.scaled_axes();
        if !forward.is_finite() || !steer.is_finite() {
            self.reset();
            return [0.0; 2];
        }
        let orientation = if self.heading.is_some() || self.anti_tip.is_some() {
            drive.tracking.refresh_orientation()
        } else {
            None
        };
        let turn = self.heading.as_mut().map_or(steer, |heading| {
            heading.turn(
                forward,
                steer,
                orientation.and_then(|sample| sample.heading),
            )
        });
        let correction = self.anti_tip.as_mut().map_or(0.0, |(axis, anti_tip)| {
            let tilt = orientation.and_then(|sample| match axis {
                TiltAxis::Pitch => sample.pitch,
                TiltAxis::Roll => sample.roll,
            });
            match tilt {
                Some(tilt) => anti_tip.correction(tilt, dt.min(MAX_CONTROL_GAP)),
                None => {
                    anti_tip.reset();
                    0.0
                }
            }
        });
        arcade_mix(forward + correction, turn)
    }
}
