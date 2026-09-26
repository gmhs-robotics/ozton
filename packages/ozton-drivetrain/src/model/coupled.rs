//! Mixed-power motors tied to the same differential drivetrain shafts.

use std::{
    rc::Rc,
    time::{Duration, Instant},
};

use ozton_control::drive::{AntiTip, SlewLimiter};
use vexide::{
    prelude::{InertialSensor, Motor},
    smart::{PortError, motor::BrakeMode},
};

use super::{DrivetrainModel, Tank};

/// Motor and its output-shaft to wheel-shaft speed ratio.
pub struct DriveMotor {
    pub motor: Motor,
    pub wheel_revolutions_per_motor_revolution: f64,
}

impl DriveMotor {
    pub const fn new(motor: Motor, wheel_revolutions_per_motor_revolution: f64) -> Self {
        Self {
            motor,
            wheel_revolutions_per_motor_revolution,
        }
    }
}

/// Commands equal wheel RPM to motors with different cartridges and external gearing.
pub struct CoupledDifferential {
    left: [DriveMotor; 3],
    right: [DriveMotor; 3],
    maximum_wheel_rpm: f64,
    left_slew: SlewLimiter,
    right_slew: SlewLimiter,
    anti_tip: Option<(Rc<InertialSensor>, AntiTip)>,
    last_update: Instant,
}

impl CoupledDifferential {
    pub fn new(
        left: [DriveMotor; 3],
        right: [DriveMotor; 3],
        maximum_wheel_rpm: f64,
        rise_per_second: f64,
        fall_per_second: f64,
    ) -> Self {
        Self {
            left,
            right,
            maximum_wheel_rpm,
            left_slew: SlewLimiter::new(rise_per_second, fall_per_second),
            right_slew: SlewLimiter::new(rise_per_second, fall_per_second),
            anti_tip: None,
            last_update: Instant::now(),
        }
    }

    pub fn with_anti_tip(mut self, imu: Rc<InertialSensor>, anti_tip: AntiTip) -> Self {
        self.anti_tip = Some((imu, anti_tip));
        self
    }

    fn drive_side(motors: &mut [DriveMotor; 3], wheel_rpm: f64) -> Result<(), PortError> {
        let mut result = Ok(());
        for drive_motor in motors {
            let motor = &mut drive_motor.motor;
            let command = if wheel_rpm.abs() < 0.5 {
                motor.brake(BrakeMode::Coast)
            } else {
                motor.set_velocity(
                    (wheel_rpm / drive_motor.wheel_revolutions_per_motor_revolution).round() as i32,
                )
            };
            if command.is_err() {
                result = command;
            }
        }
        result
    }

    pub fn coast_now(&mut self) -> Result<(), PortError> {
        self.left_slew.reset();
        self.right_slew.reset();
        let left = Self::drive_side(&mut self.left, 0.0);
        let right = Self::drive_side(&mut self.right, 0.0);
        left.and(right)
    }

    pub fn is_gliding(&self) -> bool {
        self.left_slew.value().abs() > 0.005 || self.right_slew.value().abs() > 0.005
    }

    pub async fn glide_to_stop(&mut self) -> Result<(), PortError> {
        while self.is_gliding() {
            let previous = (self.left_slew.value(), self.right_slew.value());
            if let Err(error) = self.drive_tank(0.0, 0.0) {
                let _ = self.coast_now();
                return Err(error);
            }
            if previous == (self.left_slew.value(), self.right_slew.value()) {
                return self.coast_now();
            }
            vexide::time::sleep(Duration::from_millis(20)).await;
        }
        self.coast_now()
    }
}

impl DrivetrainModel for CoupledDifferential {
    type Error = PortError;
}

impl Tank for CoupledDifferential {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        let now = Instant::now();
        let dt = now
            .saturating_duration_since(self.last_update)
            .min(Duration::from_millis(100));
        self.last_update = now;
        let correction = self
            .anti_tip
            .as_mut()
            .and_then(|(imu, controller)| {
                imu.euler()
                    .ok()
                    .map(|angles| controller.correction(angles.a.as_radians(), dt))
            })
            .unwrap_or(0.0);
        let left = self
            .left_slew
            .update((left + correction).clamp(-1.0, 1.0), dt);
        let right = self
            .right_slew
            .update((right + correction).clamp(-1.0, 1.0), dt);
        let left_result = Self::drive_side(&mut self.left, left * self.maximum_wheel_rpm);
        let right_result = Self::drive_side(&mut self.right, right * self.maximum_wheel_rpm);
        left_result.and(right_result)
    }

    fn stop_now(&mut self) -> Result<(), PortError> {
        self.coast_now()
    }
}
