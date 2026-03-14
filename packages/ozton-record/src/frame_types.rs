use std::ops::{Deref, DerefMut};

use async_trait::async_trait;
use ozton_drivetrain::{
    Drivetrain,
    model::{Arcade, Differential, DrivetrainModel, Tank},
};
use ozton_tracking::{
    GpsTracking, NoTracking, Tracking, TracksHeading, TracksPosition, TracksVelocity,
    wheeled::WheeledTracking,
};
use rkyv::{Archive, Deserialize, Serialize};
use vexide::{
    adi::digital::LogicLevel,
    prelude::{AdiDigitalOut, AdiMotor, AdiPwmOut, AdiServo, Motor},
    smart::{PortError, motor::MotorControl},
};

use crate::frame::RecordMode;

/// A value that can be interpolated between two samples.
pub trait Interpolate: Clone {
    /// Interpolates `Self` between `from` and `to`.
    ///
    /// `amount` is clamped to `[0.0, 1.0]`.
    fn interpolate(from: &Self, to: &Self, amount: f64) -> Self;
}

macro_rules! impl_numeric_interpolate {
    ($($ty:ty),* $(,)?) => {
        $(
            impl Interpolate for $ty {
                fn interpolate(from: &Self, to: &Self, amount: f64) -> Self {
                    let amount = amount.clamp(0.0, 1.0);
                    (*from as f64 + ((*to as f64) - (*from as f64)) * amount).round() as $ty
                }
            }
        )*
    };
}

impl Interpolate for f64 {
    fn interpolate(from: &Self, to: &Self, amount: f64) -> Self {
        let amount = amount.clamp(0.0, 1.0);
        from + (to - from) * amount
    }
}

impl Interpolate for f32 {
    fn interpolate(from: &Self, to: &Self, amount: f64) -> Self {
        let amount = amount.clamp(0.0, 1.0) as f32;
        from + (to - from) * amount
    }
}

impl_numeric_interpolate!(i8, i16, i32, i64, i128, isize);
impl_numeric_interpolate!(u8, u16, u32, u64, u128, usize);

impl Interpolate for bool {
    fn interpolate(from: &Self, to: &Self, amount: f64) -> Self {
        if amount < 0.5 { *from } else { *to }
    }
}

impl<T: Interpolate, const N: usize> Interpolate for [T; N] {
    fn interpolate(from: &Self, to: &Self, amount: f64) -> Self {
        std::array::from_fn(|index| T::interpolate(&from[index], &to[index], amount))
    }
}

/// Associates a live field type with the type stored in a recording frame.
pub trait FrameType {
    type Output;
}

/// A live robot field that can enrich recorded data and apply it during driver control or playback.
#[async_trait(?Send)]
pub trait RecordField {
    type Output: Interpolate + Default + Clone;

    /// Finalizes a just-sampled frame value before it is written to the recording.
    ///
    /// Most fields simply clone the sampled value. Tracked devices may attach live state.
    async fn finalize_frame_value(&self, frame: &Self::Output) -> Self::Output {
        frame.clone()
    }

    /// Applies a frame value according to the current recorder mode.
    async fn apply_frame_value(
        &mut self,
        frame: &Self::Output,
        mode: RecordMode,
    ) -> Result<(), PortError>;
}

impl<T: RecordField> FrameType for T {
    type Output = T::Output;
}

/// Opts a tracking type into `RecordableDrivetrain<Differential, T>`.
///
/// `NoTracking` and `()` replay raw voltages. Tracking implementations that can provide
/// position/heading/velocity should return a motion snapshot and choose an appropriate default
/// playback mode.
pub trait DifferentialRecording: Tracking {
    fn default_differential_playback() -> DifferentialPlayback {
        DifferentialPlayback::RawVoltage
    }

    fn tracked_motion_frame(&self) -> Option<TrackedMotionFrame> {
        None
    }
}

/// Serializable 2D position used by tracked drivetrain recordings.
#[derive(Archive, Serialize, Deserialize, Default, Clone, Copy, Debug, PartialEq)]
#[rkyv(crate = ::rkyv)]
pub struct Vec2Frame {
    pub x: f64,
    pub y: f64,
}

impl Vec2Frame {
    #[must_use]
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

impl Interpolate for Vec2Frame {
    fn interpolate(from: &Self, to: &Self, amount: f64) -> Self {
        Self {
            x: f64::interpolate(&from.x, &to.x, amount),
            y: f64::interpolate(&from.y, &to.y, amount),
        }
    }
}

/// Tracked robot motion snapshot used to correct recorded drivetrain commands.
#[derive(Archive, Serialize, Deserialize, Default, Clone, Copy, Debug, PartialEq)]
#[rkyv(crate = ::rkyv)]
pub struct TrackedMotionFrame {
    pub position: Vec2Frame,
    pub heading_radians: f64,
    pub linear_velocity: f64,
    pub angular_velocity: f64,
}

impl Interpolate for TrackedMotionFrame {
    fn interpolate(from: &Self, to: &Self, amount: f64) -> Self {
        Self {
            position: Vec2Frame::interpolate(&from.position, &to.position, amount),
            heading_radians: lerp_angle(from.heading_radians, to.heading_radians, amount),
            linear_velocity: f64::interpolate(&from.linear_velocity, &to.linear_velocity, amount),
            angular_velocity: f64::interpolate(
                &from.angular_velocity,
                &to.angular_velocity,
                amount,
            ),
        }
    }
}

/// Normalized left/right differential command.
#[derive(Archive, Serialize, Deserialize, Default, Clone, Copy, Debug, PartialEq)]
#[rkyv(crate = ::rkyv)]
pub struct DifferentialCommandFrame {
    pub left: f64,
    pub right: f64,
}

impl DifferentialCommandFrame {
    #[must_use]
    pub const fn tank(left: f64, right: f64) -> Self {
        Self { left, right }
    }

    #[must_use]
    pub fn arcade(throttle: f64, turn: f64) -> Self {
        let [left, right] = desaturate_pair(throttle + turn, throttle - turn, 1.0);
        Self { left, right }
    }
}

impl Interpolate for DifferentialCommandFrame {
    fn interpolate(from: &Self, to: &Self, amount: f64) -> Self {
        Self {
            left: f64::interpolate(&from.left, &to.left, amount),
            right: f64::interpolate(&from.right, &to.right, amount),
        }
    }
}

/// Recorded differential command with tracked motion attached for corrected playback.
#[derive(Archive, Serialize, Deserialize, Default, Clone, Copy, Debug, PartialEq)]
#[rkyv(crate = ::rkyv)]
pub struct DifferentialVoltageFrame {
    pub left: f64,
    pub right: f64,
    pub motion: TrackedMotionFrame,
}

impl DifferentialVoltageFrame {
    #[must_use]
    pub fn tank(left: f64, right: f64) -> Self {
        Self {
            left,
            right,
            ..Default::default()
        }
    }

    #[must_use]
    pub fn arcade(throttle: f64, turn: f64) -> Self {
        let [left, right] = desaturate_pair(throttle + turn, throttle - turn, 1.0);
        Self::tank(left, right)
    }
}

impl Interpolate for DifferentialVoltageFrame {
    fn interpolate(from: &Self, to: &Self, amount: f64) -> Self {
        Self {
            left: f64::interpolate(&from.left, &to.left, amount),
            right: f64::interpolate(&from.right, &to.right, amount),
            motion: TrackedMotionFrame::interpolate(&from.motion, &to.motion, amount),
        }
    }
}

/// Playback behavior for tracked differential drivetrain recordings.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum DifferentialPlayback {
    /// Replay the recorded open-loop left/right voltages directly.
    #[default]
    RawVoltage,
    /// Replay recorded voltages while correcting drift from tracked motion.
    VoltageCorrection(DifferentialVoltagePlayback),
    /// Ignore recorded voltages during playback and regenerate them from the tracked route.
    MotionTracking(DifferentialVoltagePlayback),
}

/// Tracking gains layered on top of a recorded differential command during playback.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DifferentialVoltagePlayback {
    /// Proportional gain from forward position error (meters) to shared left/right correction.
    pub longitudinal_gain: f64,

    /// Proportional gain from lateral position error (meters) to differential correction.
    pub lateral_gain: f64,

    /// Proportional gain from heading error (radians) to differential correction.
    pub heading_gain: f64,

    /// Gain from linear velocity error (m/s) to shared left/right correction.
    pub linear_velocity_gain: f64,

    /// Gain from angular velocity error (rad/s) to differential correction.
    pub angular_velocity_gain: f64,

    /// Maximum absolute shared left/right correction.
    pub max_drive_correction: f64,

    /// Maximum absolute differential correction.
    pub max_turn_correction: f64,
}

impl Default for DifferentialVoltagePlayback {
    fn default() -> Self {
        Self {
            longitudinal_gain: 1.2,
            lateral_gain: 2.0,
            heading_gain: 1.8,
            linear_velocity_gain: 0.15,
            angular_velocity_gain: 0.1,
            max_drive_correction: 0.35,
            max_turn_correction: 0.45,
        }
    }
}

/// Wrapper around a [`Drivetrain`] that adds recording/playback behavior while still dereferencing
/// to the underlying drivetrain for manual autonomous code and motion helpers.
pub struct RecordableDrivetrain<M: DrivetrainModel, T: Tracking> {
    pub drivetrain: Drivetrain<M, T>,
    differential_playback: DifferentialPlayback,
}

impl<M: DrivetrainModel, T: Tracking> RecordableDrivetrain<M, T> {
    #[must_use]
    pub const fn raw(drivetrain: Drivetrain<M, T>) -> Self {
        Self {
            drivetrain,
            differential_playback: DifferentialPlayback::RawVoltage,
        }
    }

    #[must_use]
    pub const fn with_differential_playback(
        drivetrain: Drivetrain<M, T>,
        playback: DifferentialPlayback,
    ) -> Self {
        Self {
            drivetrain,
            differential_playback: playback,
        }
    }

    #[must_use]
    pub fn into_inner(self) -> Drivetrain<M, T> {
        self.drivetrain
    }
}

impl<T: DifferentialRecording> RecordableDrivetrain<Differential, T> {
    pub fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        self.drivetrain.model.drive_tank(left, right)
    }

    pub fn drive_arcade(&mut self, throttle: f64, turn: f64) -> Result<(), PortError> {
        self.drivetrain.model.drive_arcade(throttle, turn)
    }

    #[must_use]
    pub const fn differential_playback(&self) -> DifferentialPlayback {
        self.differential_playback
    }

    pub fn set_differential_playback(&mut self, playback: DifferentialPlayback) {
        self.differential_playback = playback;
    }

    #[must_use]
    pub fn new(drivetrain: Drivetrain<Differential, T>) -> Self {
        Self::with_differential_playback(drivetrain, T::default_differential_playback())
    }

    pub fn use_voltage_correction(&mut self, playback: DifferentialVoltagePlayback) {
        self.differential_playback = DifferentialPlayback::VoltageCorrection(playback);
    }

    pub fn use_motion_tracking(&mut self, playback: DifferentialVoltagePlayback) {
        self.differential_playback = DifferentialPlayback::MotionTracking(playback);
    }
}

impl<M: DrivetrainModel, T: Tracking> Deref for RecordableDrivetrain<M, T> {
    type Target = Drivetrain<M, T>;

    fn deref(&self) -> &Self::Target {
        &self.drivetrain
    }
}

impl<M: DrivetrainModel, T: Tracking> DerefMut for RecordableDrivetrain<M, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.drivetrain
    }
}

#[async_trait(?Send)]
impl RecordField for Motor {
    type Output = f64;

    async fn apply_frame_value(
        &mut self,
        frame: &Self::Output,
        _mode: RecordMode,
    ) -> Result<(), PortError> {
        let voltage = frame.clamp(-1.0, 1.0) * self.max_voltage();
        crate::log!(
            "frame_types.motor.apply: normalized={:.4} voltage={:.4}",
            frame,
            voltage
        );
        let result = self.set_voltage(voltage);
        if let Err(error) = &result {
            crate::log!("frame_types.motor.apply: error={error:?}");
        }
        result
    }
}

#[async_trait(?Send)]
impl RecordField for AdiDigitalOut {
    type Output = bool;

    async fn apply_frame_value(
        &mut self,
        frame: &Self::Output,
        _mode: RecordMode,
    ) -> Result<(), PortError> {
        let level = if *frame {
            LogicLevel::High
        } else {
            LogicLevel::Low
        };
        crate::log!(
            "frame_types.adi_digital_out.apply: frame={} level={level:?}",
            frame
        );
        let result = self.set_level(level);
        if let Err(error) = &result {
            crate::log!("frame_types.adi_digital_out.apply: error={error:?}");
        }
        result
    }
}

#[async_trait(?Send)]
impl RecordField for AdiMotor {
    type Output = f64;

    async fn apply_frame_value(
        &mut self,
        frame: &Self::Output,
        _mode: RecordMode,
    ) -> Result<(), PortError> {
        let command = frame.clamp(-1.0, 1.0);
        crate::log!("frame_types.adi_motor.apply: normalized={command:.4}");
        let result = self.set_output(command);
        if let Err(error) = &result {
            crate::log!("frame_types.adi_motor.apply: error={error:?}");
        }
        result
    }
}

#[async_trait(?Send)]
impl RecordField for AdiServo {
    type Output = f64;

    async fn apply_frame_value(
        &mut self,
        frame: &Self::Output,
        _mode: RecordMode,
    ) -> Result<(), PortError> {
        let command = normalized_adi_output(*frame);
        crate::log!(
            "frame_types.adi_servo.apply: normalized={:.4} raw={command}",
            frame
        );
        let result = self.set_raw_target(command);
        if let Err(error) = &result {
            crate::log!("frame_types.adi_servo.apply: error={error:?}");
        }
        result
    }
}

#[async_trait(?Send)]
impl RecordField for AdiPwmOut {
    type Output = u8;

    async fn apply_frame_value(
        &mut self,
        frame: &Self::Output,
        _mode: RecordMode,
    ) -> Result<(), PortError> {
        crate::log!("frame_types.adi_pwm.apply: output={}", frame);
        let result = self.set_output(*frame);
        if let Err(error) = &result {
            crate::log!("frame_types.adi_pwm.apply: error={error:?}");
        }
        result
    }
}

#[async_trait(?Send)]
impl<const N: usize> RecordField for [Motor; N] {
    type Output = f64;

    async fn apply_frame_value(
        &mut self,
        frame: &Self::Output,
        _mode: RecordMode,
    ) -> Result<(), PortError> {
        let mut result = Ok(());
        let command = frame.clamp(-1.0, 1.0);
        crate::log!(
            "frame_types.motor_array.apply: len={} normalized={command:.4}",
            N
        );

        for motor in self {
            if let Err(error) = motor.set_voltage(command * motor.max_voltage()) {
                crate::log!("frame_types.motor_array.apply: element error={error:?}");
                result = Err(error);
            }
        }

        result
    }
}

#[async_trait(?Send)]
impl<const N: usize> RecordField for [AdiDigitalOut; N] {
    type Output = bool;

    async fn apply_frame_value(
        &mut self,
        frame: &Self::Output,
        _mode: RecordMode,
    ) -> Result<(), PortError> {
        let mut result = Ok(());
        let level = logic_level(*frame);
        crate::log!(
            "frame_types.adi_digital_out_array.apply: len={} frame={} level={level:?}",
            N,
            frame
        );

        for digital_out in self {
            if let Err(error) = digital_out.set_level(level) {
                crate::log!("frame_types.adi_digital_out_array.apply: element error={error:?}");
                result = Err(error);
            }
        }

        result
    }
}

#[async_trait(?Send)]
impl<const N: usize> RecordField for [AdiMotor; N] {
    type Output = f64;

    async fn apply_frame_value(
        &mut self,
        frame: &Self::Output,
        _mode: RecordMode,
    ) -> Result<(), PortError> {
        let mut result = Ok(());
        let command = frame.clamp(-1.0, 1.0);
        crate::log!(
            "frame_types.adi_motor_array.apply: len={} normalized={command:.4}",
            N
        );

        for motor in self {
            if let Err(error) = motor.set_output(command) {
                crate::log!("frame_types.adi_motor_array.apply: element error={error:?}");
                result = Err(error);
            }
        }

        result
    }
}

#[async_trait(?Send)]
impl<const N: usize> RecordField for [AdiServo; N] {
    type Output = f64;

    async fn apply_frame_value(
        &mut self,
        frame: &Self::Output,
        _mode: RecordMode,
    ) -> Result<(), PortError> {
        let mut result = Ok(());
        let command = normalized_adi_output(*frame);
        crate::log!(
            "frame_types.adi_servo_array.apply: len={} normalized={:.4} raw={command}",
            N,
            frame
        );

        for servo in self {
            if let Err(error) = servo.set_raw_target(command) {
                crate::log!("frame_types.adi_servo_array.apply: element error={error:?}");
                result = Err(error);
            }
        }

        result
    }
}

#[async_trait(?Send)]
impl<const N: usize> RecordField for [AdiPwmOut; N] {
    type Output = u8;

    async fn apply_frame_value(
        &mut self,
        frame: &Self::Output,
        _mode: RecordMode,
    ) -> Result<(), PortError> {
        let mut result = Ok(());
        crate::log!(
            "frame_types.adi_pwm_array.apply: len={} output={}",
            N,
            frame
        );

        for pwm_out in self {
            if let Err(error) = pwm_out.set_output(*frame) {
                crate::log!("frame_types.adi_pwm_array.apply: element error={error:?}");
                result = Err(error);
            }
        }

        result
    }
}

#[async_trait(?Send)]
impl<T> RecordField for RecordableDrivetrain<Differential, T>
where
    T: DifferentialRecording,
{
    type Output = DifferentialVoltageFrame;

    async fn finalize_frame_value(&self, frame: &Self::Output) -> Self::Output {
        let mut out = *frame;
        if let Some(motion) = self.drivetrain.tracking.tracked_motion_frame() {
            out.motion = motion;
            crate::log!(
                "frame_types.recordable_drivetrain.finalize: input={frame:?} motion={motion:?} output={out:?}"
            );
        } else {
            crate::log!(
                "frame_types.recordable_drivetrain.finalize: input={frame:?} no_tracking_motion output={out:?}"
            );
        }
        out
    }

    async fn apply_frame_value(
        &mut self,
        frame: &Self::Output,
        mode: RecordMode,
    ) -> Result<(), PortError> {
        crate::log!(
            "frame_types.recordable_drivetrain.apply: mode={mode:?} playback={:?} frame={frame:?}",
            self.differential_playback
        );
        match mode {
            RecordMode::Live => self.drivetrain.model.drive_tank(frame.left, frame.right),
            RecordMode::Playback => match (
                self.differential_playback,
                self.drivetrain.tracking.tracked_motion_frame(),
            ) {
                (DifferentialPlayback::RawVoltage, _) | (_, None) => {
                    crate::log!(
                        "frame_types.recordable_drivetrain.apply: raw voltage path left={:.4} right={:.4}",
                        frame.left,
                        frame.right
                    );
                    self.drivetrain.model.drive_tank(frame.left, frame.right)
                }
                (DifferentialPlayback::VoltageCorrection(playback), Some(current)) => {
                    crate::log!(
                        "frame_types.recordable_drivetrain.apply: voltage correction current={current:?} target_motion={:?}",
                        frame.motion
                    );
                    apply_tracked_differential_motion(
                        &mut self.drivetrain.model,
                        current,
                        frame,
                        playback,
                        true,
                    )
                }
                (DifferentialPlayback::MotionTracking(playback), Some(current)) => {
                    crate::log!(
                        "frame_types.recordable_drivetrain.apply: motion tracking current={current:?} target_motion={:?}",
                        frame.motion
                    );
                    apply_tracked_differential_motion(
                        &mut self.drivetrain.model,
                        current,
                        frame,
                        playback,
                        false,
                    )
                }
            },
        }
    }
}

/// Replays a recorded differential command and optionally uses tracking to correct drift.
pub fn apply_tracked_differential_frame<T>(
    drivetrain: &mut Drivetrain<Differential, T>,
    playback: DifferentialPlayback,
    target: &DifferentialVoltageFrame,
) -> Result<(), PortError>
where
    T: DifferentialRecording,
{
    crate::log!(
        "frame_types.apply_tracked_differential_frame: playback={playback:?} target={target:?}"
    );
    let Some(current) = drivetrain.tracking.tracked_motion_frame() else {
        crate::log!(
            "frame_types.apply_tracked_differential_frame: no tracked motion available, falling back to raw voltage"
        );
        return drivetrain.model.drive_tank(target.left, target.right);
    };

    match playback {
        DifferentialPlayback::RawVoltage => drivetrain.model.drive_tank(target.left, target.right),
        DifferentialPlayback::VoltageCorrection(playback) => apply_tracked_differential_motion(
            &mut drivetrain.model,
            current,
            target,
            playback,
            true,
        ),
        DifferentialPlayback::MotionTracking(playback) => apply_tracked_differential_motion(
            &mut drivetrain.model,
            current,
            target,
            playback,
            false,
        ),
    }
}

/// Backwards-compatible helper for the tracked-voltage playback mode.
pub fn apply_differential_voltage_frame<T>(
    drivetrain: &mut Drivetrain<Differential, T>,
    playback: DifferentialVoltagePlayback,
    target: &DifferentialVoltageFrame,
) -> Result<(), PortError>
where
    T: DifferentialRecording,
{
    apply_tracked_differential_frame(
        drivetrain,
        DifferentialPlayback::VoltageCorrection(playback),
        target,
    )
}

fn apply_tracked_differential_motion(
    model: &mut Differential,
    current: TrackedMotionFrame,
    target: &DifferentialVoltageFrame,
    playback: DifferentialVoltagePlayback,
    include_recorded_voltage: bool,
) -> Result<(), PortError> {
    let (drive_correction, turn_correction) =
        tracked_playback_correction(&current, &target.motion, playback);

    let (drive_base, turn_base) = if include_recorded_voltage {
        (
            (target.left + target.right) / 2.0,
            (target.left - target.right) / 2.0,
        )
    } else {
        (0.0, 0.0)
    };

    let [left, right] = desaturate_pair(
        drive_base + drive_correction + turn_base + turn_correction,
        drive_base + drive_correction - (turn_base + turn_correction),
        1.0,
    );

    crate::log!(
        "frame_types.apply_tracked_differential_motion: current={current:?} target={target:?} include_recorded_voltage={} drive_base={drive_base:.4} turn_base={turn_base:.4} drive_correction={drive_correction:.4} turn_correction={turn_correction:.4} left={left:.4} right={right:.4}",
        include_recorded_voltage
    );
    model.drive_tank(left, right)
}

fn tracked_motion_frame<T>(tracking: &T) -> TrackedMotionFrame
where
    T: TracksPosition + TracksHeading + TracksVelocity,
{
    let position = tracking.position();

    TrackedMotionFrame {
        position: Vec2Frame::new(position.x, position.y),
        heading_radians: tracking.heading().as_radians(),
        linear_velocity: tracking.linear_velocity(),
        angular_velocity: tracking.angular_velocity(),
    }
}

impl DifferentialRecording for NoTracking {}

impl DifferentialRecording for () {}

impl DifferentialRecording for GpsTracking {
    fn default_differential_playback() -> DifferentialPlayback {
        DifferentialPlayback::VoltageCorrection(DifferentialVoltagePlayback::default())
    }

    fn tracked_motion_frame(&self) -> Option<TrackedMotionFrame> {
        Some(tracked_motion_frame(self))
    }
}

impl DifferentialRecording for WheeledTracking {
    fn default_differential_playback() -> DifferentialPlayback {
        DifferentialPlayback::VoltageCorrection(DifferentialVoltagePlayback::default())
    }

    fn tracked_motion_frame(&self) -> Option<TrackedMotionFrame> {
        Some(tracked_motion_frame(self))
    }
}

fn tracked_playback_correction(
    current: &TrackedMotionFrame,
    target: &TrackedMotionFrame,
    playback: DifferentialVoltagePlayback,
) -> (f64, f64) {
    let dx = target.position.x - current.position.x;
    let dy = target.position.y - current.position.y;
    let heading = current.heading_radians;

    // Rotate field error into the robot frame so forward and lateral corrections stay decoupled.
    let local_x = dx * heading.cos() + dy * heading.sin();
    let local_y = -dx * heading.sin() + dy * heading.cos();
    let heading_error = wrap_radians(target.heading_radians - current.heading_radians);
    let linear_velocity_error = target.linear_velocity - current.linear_velocity;
    let angular_velocity_error = target.angular_velocity - current.angular_velocity;

    let mut drive_correction = playback.longitudinal_gain * local_x
        + playback.linear_velocity_gain * linear_velocity_error;
    drive_correction *= heading_error.cos().max(0.0);

    let turn_correction = playback.lateral_gain * local_y
        + playback.heading_gain * heading_error
        + playback.angular_velocity_gain * angular_velocity_error;

    let output = (
        drive_correction.clamp(
            -playback.max_drive_correction,
            playback.max_drive_correction,
        ),
        turn_correction.clamp(-playback.max_turn_correction, playback.max_turn_correction),
    );

    crate::log!(
        "frame_types.tracked_playback_correction: current={current:?} target={target:?} dx={dx:.4} dy={dy:.4} local_x={local_x:.4} local_y={local_y:.4} heading_error={heading_error:.4} linear_velocity_error={linear_velocity_error:.4} angular_velocity_error={angular_velocity_error:.4} drive_out={:.4} turn_out={:.4}",
        output.0,
        output.1
    );

    output
}

fn desaturate_pair(left: f64, right: f64, max: f64) -> [f64; 2] {
    let largest_magnitude = left.abs().max(right.abs());

    if largest_magnitude > max {
        [
            left * max / largest_magnitude,
            right * max / largest_magnitude,
        ]
    } else {
        [left, right]
    }
}

fn wrap_radians(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

fn lerp_angle(from: f64, to: f64, amount: f64) -> f64 {
    from + wrap_radians(to - from) * amount.clamp(0.0, 1.0)
}

fn logic_level(value: bool) -> LogicLevel {
    if value {
        LogicLevel::High
    } else {
        LogicLevel::Low
    }
}

fn normalized_adi_output(value: f64) -> i8 {
    (value.clamp(-1.0, 1.0) * f64::from(i8::MAX)).round() as i8
}

#[allow(dead_code)]
fn normalized_motor_output(motor: &Motor) -> f64 {
    let max_voltage = motor.max_voltage();
    if max_voltage == 0.0 {
        return 0.0;
    }

    let commanded = match motor.target() {
        MotorControl::Voltage(voltage) => voltage,
        MotorControl::Brake(_) => 0.0,
        _ => motor.voltage().unwrap_or_default(),
    };

    (commanded / max_voltage).clamp(-1.0, 1.0)
}
