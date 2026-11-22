//! Differential voltage output and optional wheel-speed feedback.

use std::time::Instant;

use ozton_control::drive::SlewLimiter;
use ozton_math::{
    desaturate,
    uom::si::{
        electric_current::ampere,
        electric_potential::volt,
        f64::{AngularVelocity, ElectricCurrent, ElectricPotential, Frequency, Length, Velocity},
        length::meter,
        velocity::meter_per_second,
    },
};
use vexide::{
    prelude::Motor,
    smart::{PortError, motor::BrakeMode},
};

use super::{Arcade, DrivetrainModel, Tank};
use crate::feedback::MAX_SIDE_MOTORS;

/// Motor-to-wheel transmission. Each stage is specified as (driver teeth,
/// driven teeth), starting at the motor's cartridge output shaft.
#[derive(Clone, Copy, Debug)]
pub struct WheelGearing(f64);

impl WheelGearing {
    pub const fn new(driver_teeth: u16, driven_teeth: u16) -> Self {
        Self(driver_teeth as f64 / driven_teeth as f64)
    }

    pub const fn then(self, driver_teeth: u16, driven_teeth: u16) -> Self {
        Self(self.0 * Self::new(driver_teeth, driven_teeth).0)
    }
}

impl From<(u16, u16)> for WheelGearing {
    fn from((driver, driven): (u16, u16)) -> Self {
        Self::new(driver, driven)
    }
}

struct DriveMotor {
    motor: Motor,
    gearing: Option<WheelGearing>,
}

#[derive(Clone, Copy)]
struct WheelGeometry {
    free_rpm: Option<f64>,
    circumference: Length,
}

impl WheelGeometry {
    fn meters_per_motor_turn(self, drive: &DriveMotor) -> Result<f64, PortError> {
        let ratio = match drive.gearing {
            Some(gearing) => gearing.0,
            None => self.free_rpm.unwrap_or_default() / drive.motor.gearset()?.max_rpm(),
        };
        Ok(self.circumference.value * ratio)
    }

    fn speed(self, motors: &mut [DriveMotor], speed: f64) -> Result<(), PortError> {
        each(motors, |drive| {
            let rpm = speed * 60.0 / self.meters_per_motor_turn(drive)?;
            drive.motor.set_velocity(rpm.round() as i32)
        })
    }

    fn feedback(
        self,
        motors: &[DriveMotor],
        consensus: &mut crate::feedback::EncoderConsensus,
        dt: std::time::Duration,
    ) -> Option<WheelFeedback> {
        if motors.len() > MAX_SIDE_MOTORS {
            return None;
        }
        let mut samples = [None; MAX_SIDE_MOTORS];
        for (slot, drive) in samples.iter_mut().zip(motors) {
            *slot = (|| {
                let meters_per_turn = self.meters_per_motor_turn(drive).ok()?;
                Some(WheelFeedback {
                    position: Length::new::<meter>(
                        drive.motor.position().ok()?.as_turns() * meters_per_turn,
                    ),
                    velocity: Velocity::new::<meter_per_second>(
                        drive.motor.velocity().ok()? * meters_per_turn / 60.0,
                    ),
                })
            })();
        }
        consensus.sample(&samples[..motors.len()], dt)
    }

    fn wheel_load(self, motors: &[DriveMotor]) -> Option<WheelLoad> {
        if motors.is_empty() {
            return None;
        }
        let mut current = 0.0;
        let mut fastest_wheel: f64 = 0.0;
        for drive in motors {
            let amps = drive.motor.current().ok()?;
            let speed =
                drive.motor.velocity().ok()? * self.meters_per_motor_turn(drive).ok()? / 60.0;
            if !amps.is_finite() || !speed.is_finite() {
                return None;
            }
            current += amps.abs();
            fastest_wheel = fastest_wheel.max(speed.abs());
        }
        Some(WheelLoad {
            mean_current: ElectricCurrent::new::<ampere>(current / motors.len() as f64),
            fastest_wheel_speed: Velocity::new::<meter_per_second>(fastest_wheel),
        })
    }
}

fn geared_side(
    motors: impl IntoIterator<Item = (Motor, impl Into<WheelGearing>)>,
) -> Option<Box<[DriveMotor]>> {
    let mut side = Vec::new();
    for (motor, gearing) in motors {
        let gearing = gearing.into();
        if !gearing.0.is_finite() || gearing.0 <= 0.0 || side.len() == MAX_SIDE_MOTORS {
            return None;
        }
        side.push(DriveMotor {
            motor,
            gearing: Some(gearing),
        });
    }
    (!side.is_empty()).then(|| side.into_boxed_slice())
}

fn motor_voltage(output: f64, maximum: f64, absolute: bool) -> f64 {
    if absolute {
        (output * Motor::V5_MAX_VOLTAGE).clamp(-maximum, maximum)
    } else {
        output * maximum
    }
}

fn each<M>(
    motors: &mut [M],
    mut operation: impl FnMut(&mut M) -> Result<(), PortError>,
) -> Result<(), PortError> {
    let mut result = Ok(());
    for motor in motors {
        result = result.and(operation(motor));
    }
    result
}

/// Continuous per-side travel and velocity from all configured motors.
#[derive(Debug, Clone, Copy)]
pub struct WheelFeedback {
    pub position: Length,
    pub velocity: Velocity,
}

/// Electrical load and fastest absolute wheel speed on one drive side.
#[derive(Debug, Clone, Copy)]
pub struct WheelLoad {
    pub mean_current: ElectricCurrent,
    pub fastest_wheel_speed: Velocity,
}

/// Wheel-speed command validation or a device write failure.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WheelSpeedError {
    InvalidCommand,
    MissingWheelGeometry,
    Port(PortError),
}

/// Driver-only choice after a failed port write. Autonomous always receives the
/// original error and stops in the runner.
#[derive(Debug, Clone, Copy)]
pub enum DriverFaultPolicy {
    KeepAvailableMotors,
    Stop,
}

/// One differential drivetrain for voltage and matched wheel-speed control.
pub struct Differential {
    sides: [Box<[DriveMotor]>; 2],
    wheels: Option<WheelGeometry>,
    track_width: Option<Length>,
    wheel_speed_limit: Option<Velocity>,
    scale: f64,
    slew: Option<[SlewLimiter; 2]>,
    previous: Instant,
    brake: BrakeMode,
    consensus: [crate::feedback::EncoderConsensus; 2],
    last_feedback: [Option<WheelFeedback>; 2],
    feedback_time: Instant,
}

impl Differential {
    /// Own the motor arrays. Mechanically couple the wheels on each side and
    /// choose gearing so every motor produces the same free wheel RPM at full
    /// rated voltage (12 V for 11 W V5 motors, 8 V for 5.5 W motors). Motors
    /// driving the same wheels at different speeds fight each other.
    pub fn new(left: impl Into<Box<[Motor]>>, right: impl Into<Box<[Motor]>>) -> Self {
        let sides = [left.into(), right.into()].map(|motors| {
            Vec::from(motors)
                .into_iter()
                .map(|motor| DriveMotor {
                    motor,
                    gearing: None,
                })
                .collect()
        });
        Self::from_sides(sides, None)
    }

    /// Configure wheel diameter and explicit gearing for every motor. Tuple ratios
    /// such as `(36, 48)` describe one stage. Use [`WheelGearing::then`] for more.
    /// Choose ratios that match wheel free speeds across the mechanically linked motors.
    pub fn with_geared_motors<L, R>(
        left: impl IntoIterator<Item = (Motor, L)>,
        right: impl IntoIterator<Item = (Motor, R)>,
        diameter: Length,
    ) -> Option<Self>
    where
        L: Into<WheelGearing>,
        R: Into<WheelGearing>,
    {
        let circumference = diameter * core::f64::consts::PI;
        let sides = [geared_side(left)?, geared_side(right)?];
        (circumference.value.is_finite() && circumference.value > 0.0).then(|| {
            Self::from_sides(
                sides,
                Some(WheelGeometry {
                    circumference,
                    free_rpm: None,
                }),
            )
        })
    }

    fn from_sides(sides: [Box<[DriveMotor]>; 2], wheels: Option<WheelGeometry>) -> Self {
        Self {
            sides,
            wheels,
            track_width: None,
            wheel_speed_limit: None,
            scale: 1.0,
            slew: None,
            previous: Instant::now(),
            brake: BrakeMode::Brake,
            consensus: core::array::from_fn(|_| crate::feedback::EncoderConsensus::default()),
            last_feedback: [None; 2],
            feedback_time: Instant::now(),
        }
    }

    /// Configure one physical wheel diameter and free wheel speed for both sides.
    /// Wheel measurements and native speed commands use the same wheel calibration.
    /// Free speed infers ratios for motors without an explicit gear train.
    pub fn with_wheel_feedback(
        mut self,
        diameter: Length,
        free_speed: AngularVelocity,
    ) -> Option<Self> {
        use ozton_math::uom::si::angular_velocity::revolution_per_minute;

        let free_rpm = free_speed.get::<revolution_per_minute>();
        let circumference = diameter * core::f64::consts::PI;
        if !free_rpm.is_finite()
            || free_rpm <= 0.0
            || !circumference.value.is_finite()
            || circumference.value <= 0.0
        {
            return None;
        }
        self.wheels = Some(WheelGeometry {
            free_rpm: Some(free_rpm),
            circumference,
        });
        Some(self)
    }

    /// Set the distance between the left and right drive contact lines.
    pub fn with_track_width(mut self, width: Length) -> Option<Self> {
        if !width.value.is_finite() || width.value <= 0.0 {
            return None;
        }
        self.track_width = Some(width);
        Some(self)
    }

    /// Calibrated distance between the left and right drive contact lines.
    pub fn track_width(&self) -> Option<Length> {
        self.track_width
    }

    /// Limit driver tank input to a physical speed and use native motor velocity control.
    pub fn with_native_speed_limit(mut self, limit: Velocity) -> Option<Self> {
        if self.wheels.is_none() || !limit.value.is_finite() || limit.value <= 0.0 {
            return None;
        }
        self.wheel_speed_limit = Some(limit);
        Some(self)
    }

    pub fn with_slew(
        mut self,
        rise_per_second: Frequency,
        fall_per_second: Frequency,
    ) -> Option<Self> {
        if ![rise_per_second.value, fall_per_second.value]
            .iter()
            .all(|rate| rate.is_finite() && *rate > 0.0)
        {
            return None;
        }
        self.slew = Some([SlewLimiter::new(rise_per_second, fall_per_second); 2]);
        Some(self)
    }

    pub fn set_output_scale(&mut self, scale: f64) {
        self.scale = if scale.is_finite() {
            scale.clamp(0.0, 1.0)
        } else {
            0.0
        };
    }

    pub fn with_brake(mut self, brake: BrakeMode) -> Self {
        self.brake = brake;
        self
    }

    pub fn wheel_speed_limit(&self) -> Option<Velocity> {
        self.wheel_speed_limit
    }

    /// Capture one encoder update and advance both side baselines together.
    pub fn sample_feedback(&mut self) -> [Option<WheelFeedback>; 2] {
        let now = Instant::now();
        let dt = now.saturating_duration_since(core::mem::replace(&mut self.feedback_time, now));
        self.last_feedback = core::array::from_fn(|i| {
            self.wheels?
                .feedback(&self.sides[i], &mut self.consensus[i], dt)
        });
        self.last_feedback
    }

    /// Return cached feedback without reading encoders or advancing the baseline.
    pub fn feedback(&self) -> [Option<WheelFeedback>; 2] {
        self.last_feedback
    }

    /// Visit motors in left/right order without sampling or advancing odometry.
    pub fn visit_motors(&self, mut visit: impl FnMut(usize, usize, &Motor)) {
        for (side, motors) in self.sides.iter().enumerate() {
            for (index, motor) in motors.iter().enumerate() {
                visit(side, index, &motor.motor);
            }
        }
    }

    /// Read each motor once for load detection without advancing odometry.
    /// A side reports None if any of its motor feedback is unavailable.
    pub fn wheel_load(&self) -> [Option<WheelLoad>; 2] {
        self.sides
            .each_ref()
            .map(|motors| self.wheels?.wheel_load(motors))
    }

    pub fn measured_voltage(&self) -> [Option<ElectricPotential>; 2] {
        self.sides.each_ref().map(|motors| {
            motors
                .first()?
                .motor
                .voltage()
                .ok()
                .map(ElectricPotential::new::<volt>)
        })
    }

    /// Stops both sides immediately, bypassing slew.
    pub fn brake_now(&mut self, brake: BrakeMode) -> Result<(), PortError> {
        if let Some(slew) = &mut self.slew {
            slew.iter_mut().for_each(SlewLimiter::reset);
        }
        self.previous = Instant::now();
        each(&mut self.sides, |side| {
            each(side, |drive| drive.motor.brake(brake))
        })
    }

    /// Applies normalized voltage even when tank input is configured for wheel speed.
    pub fn drive_normalized_voltage(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        self.apply([left, right], None, false)
    }

    /// Applies volts to every motor on each side, limited by each motor's rating.
    /// Optional output scaling and slew limiting still apply.
    pub fn drive_volts(
        &mut self,
        left: ElectricPotential,
        right: ElectricPotential,
    ) -> Result<(), PortError> {
        let maximum = Motor::V5_MAX_VOLTAGE;
        self.apply([left.value / maximum, right.value / maximum], None, true)
    }

    /// SI wheel-speed control without changing the driver's voltage-control mode.
    /// Requires wheel calibration. Scaling, slew and anti-tip remain active.
    pub fn drive_wheel_speeds(
        &mut self,
        left: Velocity,
        right: Velocity,
        maximum: Velocity,
    ) -> Result<(), WheelSpeedError> {
        let (left, right, maximum) = (left.value, right.value, maximum.value);
        if !maximum.is_finite() || maximum <= 0.0 || !left.is_finite() || !right.is_finite() {
            let _ = self.brake_now(self.brake);
            return Err(WheelSpeedError::InvalidCommand);
        }
        if self.wheels.is_none() {
            let _ = self.brake_now(self.brake);
            return Err(WheelSpeedError::MissingWheelGeometry);
        }
        self.apply([left / maximum, right / maximum], Some(maximum), false)
            .map_err(WheelSpeedError::Port)
    }

    pub fn drive_driver_arcade(
        &mut self,
        throttle: f64,
        steer: f64,
        policy: DriverFaultPolicy,
    ) -> Result<(), PortError> {
        let output = self.drive_arcade(throttle, steer);
        if output.is_err() && matches!(policy, DriverFaultPolicy::Stop) {
            let _ = self.stop_now();
        }
        output
    }

    fn apply(
        &mut self,
        input: [f64; 2],
        limit: Option<f64>,
        absolute: bool,
    ) -> Result<(), PortError> {
        if input.iter().any(|value| !value.is_finite()) {
            return self.stop_now();
        }
        let now = Instant::now();
        let dt = now
            .saturating_duration_since(self.previous)
            .min(std::time::Duration::from_millis(100));
        self.previous = now;
        let mut command = desaturate(input, 1.0);
        if let Some(slew) = &mut self.slew {
            command = [
                slew[0].update(command[0], dt),
                slew[1].update(command[1], dt),
            ];
        }
        let [left, right] = &mut self.sides;
        let [left_power, right_power] = command.map(|power| power * self.scale);
        if let (Some(limit), Some(wheels)) = (limit, self.wheels) {
            wheels
                .speed(left, left_power * limit)
                .and(wheels.speed(right, right_power * limit))
        } else {
            let set_voltage = |motors: &mut [DriveMotor], power| {
                each(motors, |drive| {
                    let motor = &mut drive.motor;
                    motor.set_voltage(motor_voltage(power, motor.max_voltage(), absolute))
                })
            };
            set_voltage(left, left_power).and(set_voltage(right, right_power))
        }
    }
}

impl DrivetrainModel for Differential {
    type Error = PortError;
}

impl Tank for Differential {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        self.apply(
            [left, right],
            self.wheel_speed_limit.map(|limit| limit.value),
            false,
        )
    }

    fn stop_now(&mut self) -> Result<(), PortError> {
        self.brake_now(self.brake)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compound_gearing_matches_blue_wheel_speed() {
        let blue = WheelGearing::from((36, 48));
        let exp = WheelGearing::new(36, 12).then(36, 48);
        assert_eq!(600.0 * blue.0, 450.0);
        assert_eq!(200.0 * exp.0, 450.0);
        assert_eq!(WheelGearing::from((84, 12)).0, 7.0);
    }

    #[test]
    fn physical_voltage_preserves_setpoint_for_mixed_motors() {
        assert_eq!(motor_voltage(0.5, 8.0, true), 6.0);
        assert_eq!(motor_voltage(0.5, 12.0, true), 6.0);
        assert_eq!(motor_voltage(-1.0, 8.0, true), -8.0);
        assert_eq!(motor_voltage(0.5, 8.0, false), 4.0);
    }
}
