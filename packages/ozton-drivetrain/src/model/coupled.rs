//! Mixed-power motors tied to the same differential drivetrain shafts.

use std::{
    rc::Rc,
    time::{Duration, Instant},
};

use ozton_control::drive::{AntiTip, SlewLimiter};
use vexide::{
    math::Angle,
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

#[derive(Clone, Copy)]
enum DriveOutput {
    WheelRpm(f64),
    Voltage,
}

/// IMU Euler angle that represents the robot's fore-aft tilt.
#[derive(Clone, Copy)]
pub enum TipAngleAxis {
    Pitch,
    Roll,
}

impl TipAngleAxis {
    fn read(self, imu: &InertialSensor) -> Option<Angle> {
        imu.euler().ok().map(|angles| match self {
            Self::Pitch => angles.a,
            Self::Roll => angles.c,
        })
    }
}

/// Drives mixed-power motors using matched wheel RPM or direct voltage.
pub struct CoupledDifferential {
    left: [DriveMotor; 3],
    right: [DriveMotor; 3],
    output: DriveOutput,
    left_slew: SlewLimiter,
    right_slew: SlewLimiter,
    direct_response: bool,
    anti_tip: Option<(Rc<InertialSensor>, AntiTip, TipAngleAxis, Angle)>,
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
            output: DriveOutput::WheelRpm(maximum_wheel_rpm),
            left_slew: SlewLimiter::new(rise_per_second, fall_per_second),
            right_slew: SlewLimiter::new(rise_per_second, fall_per_second),
            direct_response: false,
            anti_tip: None,
            last_update: Instant::now(),
        }
    }

    pub fn with_anti_tip(mut self, imu: Rc<InertialSensor>, anti_tip: AntiTip) -> Self {
        self.anti_tip = Some((imu, anti_tip, TipAngleAxis::Pitch, Angle::ZERO));
        self
    }

    /// Uses a chosen IMU axis and its level reading for tilt correction.
    pub fn with_anti_tip_axis(
        mut self,
        imu: Rc<InertialSensor>,
        anti_tip: AntiTip,
        axis: TipAngleAxis,
        level: Angle,
    ) -> Self {
        self.anti_tip = Some((imu, anti_tip, axis, level));
        self
    }

    /// Creates a drivetrain that applies requested wheel speed without a command ramp.
    pub fn new_direct(
        left: [DriveMotor; 3],
        right: [DriveMotor; 3],
        maximum_wheel_rpm: f64,
    ) -> Self {
        let mut model = Self::new(left, right, maximum_wheel_rpm, 0.0, 0.0);
        model.direct_response = true;
        model
    }

    /// Applies full rated voltage to every motor at a full-scale command.
    pub fn new_direct_voltage(left: [DriveMotor; 3], right: [DriveMotor; 3]) -> Self {
        let mut model = Self::new_direct(left, right, 0.0);
        model.output = DriveOutput::Voltage;
        model
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

    fn drive_side_voltage(motors: &mut [DriveMotor; 3], power: f64) -> Result<(), PortError> {
        let mut result = Ok(());
        for drive_motor in motors {
            let motor = &mut drive_motor.motor;
            if let Err(error) = motor.set_voltage(power * motor.max_voltage()) {
                result = Err(error);
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
        !self.direct_response
            && (self.left_slew.value().abs() > 0.005 || self.right_slew.value().abs() > 0.005)
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
            .and_then(|(imu, controller, axis, level)| {
                axis.read(imu).map(|angle| {
                    controller.correction((angle - *level).wrapped_half().as_radians(), dt)
                })
            })
            .unwrap_or(0.0);
        let left = (left + correction).clamp(-1.0, 1.0);
        let right = (right + correction).clamp(-1.0, 1.0);
        let (left, right) = if self.direct_response {
            (left, right)
        } else {
            (
                self.left_slew.update(left, dt),
                self.right_slew.update(right, dt),
            )
        };
        let (left_result, right_result) = match self.output {
            DriveOutput::WheelRpm(maximum) => (
                Self::drive_side(&mut self.left, left * maximum),
                Self::drive_side(&mut self.right, right * maximum),
            ),
            DriveOutput::Voltage => (
                Self::drive_side_voltage(&mut self.left, left),
                Self::drive_side_voltage(&mut self.right, right),
            ),
        };
        left_result.and(right_result)
    }

    fn stop_now(&mut self) -> Result<(), PortError> {
        self.coast_now()
    }
}
