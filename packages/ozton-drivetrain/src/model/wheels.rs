//! Motor gearing and physical wheel feedback shared by drivetrain models.
use ozton_math::uom::si::{
    electric_current::ampere,
    f64::{ElectricCurrent, Length, Velocity},
    length::meter,
    velocity::meter_per_second,
};
use vexide::{prelude::Motor, smart::PortError};

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

/// An owned motor with optional explicit wheel gearing.
pub struct DriveMotor {
    pub(super) motor: Motor,
    pub(super) gearing: Option<WheelGearing>,
}

impl From<Motor> for DriveMotor {
    fn from(motor: Motor) -> Self {
        Self {
            motor,
            gearing: None,
        }
    }
}

impl<G: Into<WheelGearing>> From<(Motor, G)> for DriveMotor {
    fn from((motor, gearing): (Motor, G)) -> Self {
        Self {
            motor,
            gearing: Some(gearing.into()),
        }
    }
}

#[derive(Clone, Copy)]
pub(super) struct WheelGeometry {
    pub(super) free_rpm: Option<f64>,
    pub(super) circumference: Length,
}

impl WheelGeometry {
    pub(super) fn new(
        diameter: Length,
        free_speed: Option<ozton_math::uom::si::f64::AngularVelocity>,
    ) -> Option<Self> {
        use ozton_math::uom::si::angular_velocity::revolution_per_minute;
        let circumference = diameter * core::f64::consts::PI;
        let free_rpm = free_speed.map(|speed| speed.get::<revolution_per_minute>());
        (circumference.value.is_finite()
            && circumference.value > 0.0
            && free_rpm.is_none_or(|rpm| rpm.is_finite() && rpm > 0.0))
        .then_some(Self {
            circumference,
            free_rpm,
        })
    }

    fn meters_per_motor_turn(self, drive: &DriveMotor) -> Result<f64, PortError> {
        let ratio = match drive.gearing {
            Some(gearing) => gearing.0,
            None => self.free_rpm.unwrap_or_default() / drive.motor.gearset()?.max_rpm(),
        };
        Ok(self.circumference.value * ratio)
    }

    pub(super) fn speed(self, motors: &mut [DriveMotor], speed: f64) -> Result<(), PortError> {
        each(motors, |drive| {
            let rpm = speed * 60.0 / self.meters_per_motor_turn(drive)?;
            drive.motor.set_velocity(rpm.round() as i32)
        })
    }

    pub(super) fn feedback(
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

    pub(super) fn wheel_load(self, motors: &[DriveMotor]) -> Option<WheelLoad> {
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

pub(super) fn valid_gearing(motors: &[DriveMotor]) -> bool {
    !motors.is_empty()
        && motors.len() <= MAX_SIDE_MOTORS
        && motors.iter().all(|motor| {
            motor
                .gearing
                .is_some_and(|ratio| ratio.0.is_finite() && ratio.0 > 0.0)
        })
}

pub(super) fn write_wheels<const N: usize>(
    groups: &mut [Box<[DriveMotor]>; N],
    powers: [f64; N],
    speed: Option<(WheelGeometry, f64)>,
    absolute: bool,
) -> Result<(), PortError> {
    let mut result = Ok(());
    for (group, power) in groups.iter_mut().zip(powers) {
        let written = if let Some((wheels, limit)) = speed {
            wheels.speed(group, power * limit)
        } else {
            each(group, |drive| {
                let motor = &mut drive.motor;
                motor.set_voltage(motor_voltage(power, motor.max_voltage(), absolute))
            })
        };
        result = result.and(written);
    }
    result
}

pub(super) fn shape<const N: usize>(
    powers: [f64; N],
    slew: &mut Option<[ozton_control::drive::SlewLimiter; N]>,
    previous: &mut std::time::Instant,
) -> [f64; N] {
    let dt = previous
        .elapsed()
        .min(std::time::Duration::from_millis(100));
    *previous = std::time::Instant::now();
    let powers = ozton_math::desaturate(powers, 1.0);
    if let Some(slew) = slew {
        core::array::from_fn(|i| slew[i].update(powers[i], dt))
    } else {
        powers
    }
}

pub(super) fn motor_voltage(output: f64, maximum: f64, absolute: bool) -> f64 {
    if absolute {
        (output * Motor::V5_MAX_VOLTAGE).clamp(-maximum, maximum)
    } else {
        output * maximum
    }
}

pub(super) fn each<M>(
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

#[cfg(any(debug_assertions, feature = "sd-logging"))]
#[derive(Clone, Copy)]
pub struct DriveSnapshot {
    pub command: [f64; 2],
    pub scale: f64,
    pub wheel_meters: [f64; 2],
    pub wheel_revolutions: [f64; 2],
    pub write_failed: bool,
    pub fault_count: u64,
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
