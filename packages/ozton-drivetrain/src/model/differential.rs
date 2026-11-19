//! Differential voltage and wheel-speed output for ordinary or mixed gearing.

use std::{cell::RefCell, rc::Rc, time::Instant};

use ozton_control::drive::{AntiTip, SlewLimiter};
use ozton_math::desaturate;
use vexide::{
    math::Angle,
    prelude::{InertialSensor, Motor},
    smart::{PortError, motor::BrakeMode},
};

use super::{Arcade, DriveMotor, DrivetrainModel, Tank};

type SharedSide = Rc<RefCell<dyn AsMut<[Motor]>>>;

enum Side {
    Shared(SharedSide),
    Geared(Vec<DriveMotor>),
}

impl Side {
    fn voltage(&mut self, output: f64, absolute: bool) -> Result<(), PortError> {
        let apply = |motor: &mut Motor| {
            motor.set_voltage(motor_voltage(output, motor.max_voltage(), absolute))
        };
        match self {
            Self::Shared(motors) => each(motors.borrow_mut().as_mut(), apply),
            Self::Geared(motors) => each(motors, |motor| apply(&mut motor.motor)),
        }
    }

    fn speed(&mut self, meters_per_second: f64) -> Result<(), PortError> {
        let Self::Geared(motors) = self else {
            unreachable!("wheel speed mode requires geared sides");
        };
        each(motors, |motor| {
            motor
                .motor
                .set_velocity((meters_per_second * motor.rpm_per_meter_per_second).round() as i32)
        })
    }

    fn stop(&mut self, brake: BrakeMode) -> Result<(), PortError> {
        match self {
            Self::Shared(motors) => each(motors.borrow_mut().as_mut(), |motor| motor.brake(brake)),
            Self::Geared(motors) => each(motors, |motor| motor.motor.brake(brake)),
        }
    }

    fn feedback(
        &self,
        consensus: &RefCell<crate::feedback::EncoderConsensus>,
        dt: std::time::Duration,
    ) -> Option<WheelFeedback> {
        let Self::Geared(motors) = self else {
            return None;
        };
        let mut samples = [None; crate::feedback::MAX_SIDE_MOTORS];
        for (slot, motor) in samples.iter_mut().zip(motors) {
            *slot = motor
                .motor
                .position()
                .ok()
                .zip(motor.motor.velocity().ok())
                .map(|(position, velocity)| WheelFeedback {
                    position: position.as_turns() * motor.meters_per_turn,
                    velocity: velocity / motor.rpm_per_meter_per_second,
                });
        }
        consensus.borrow_mut().sample(&samples[..motors.len()], dt)
    }

    fn wheel_load(&self) -> Option<WheelLoad> {
        let Self::Geared(motors) = self else {
            return None;
        };
        let mut current = 0.0;
        let mut fastest_wheel: f64 = 0.0;
        for motor in motors {
            let amps = motor.motor.current().ok()?;
            let speed = motor.motor.velocity().ok()? / motor.rpm_per_meter_per_second;
            if !amps.is_finite() || !speed.is_finite() {
                return None;
            }
            current += amps.abs();
            fastest_wheel = fastest_wheel.max(speed.abs());
        }
        Some(WheelLoad {
            mean_current_amps: current / motors.len() as f64,
            fastest_wheel_mps: fastest_wheel,
        })
    }

    fn voltage_feedback(&self) -> Option<f64> {
        match self {
            Self::Geared(motors) => motors.first()?.motor.voltage().ok(),
            Self::Shared(motors) => motors.borrow_mut().as_mut().first()?.voltage().ok(),
        }
    }
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

/// Continuous per-side travel and velocity from all configured geared motors.
#[derive(Debug, Clone, Copy)]
pub struct WheelFeedback {
    pub position: f64,
    pub velocity: f64,
}

/// Electrical load and fastest absolute wheel speed on one geared drive side.
#[derive(Debug, Clone, Copy)]
pub struct WheelLoad {
    pub mean_current_amps: f64,
    pub fastest_wheel_mps: f64,
}

/// Wheel-speed command validation or a device write failure.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WheelSpeedError {
    InvalidCommand,
    NotGeared,
    Port(PortError),
}

/// Measured fore-aft tilt axis.
#[derive(Debug, Clone, Copy)]
pub enum TipAngleAxis {
    Pitch,
    Roll,
}

/// Driver-only choice after a failed port write. Autonomous always receives the
/// original error and stops in the runner.
#[derive(Debug, Clone, Copy)]
pub enum DriverFaultPolicy {
    KeepAvailableMotors,
    Stop,
}

struct TipControl {
    imu: Rc<InertialSensor>,
    controller: AntiTip,
    axis: TipAngleAxis,
    level: Angle,
}

/// One differential drivetrain for voltage and matched wheel-speed control.
pub struct Differential {
    sides: [Side; 2],
    wheel_speed_limit: Option<f64>,
    scale: f64,
    slew: Option<[SlewLimiter; 2]>,
    anti_tip: Option<TipControl>,
    previous: Instant,
    brake: BrakeMode,
    feedback: [RefCell<crate::feedback::EncoderConsensus>; 2],
    feedback_time: std::cell::Cell<Instant>,
}

impl Differential {
    pub fn new<L: AsMut<[Motor]> + 'static, R: AsMut<[Motor]> + 'static>(
        left: L,
        right: R,
    ) -> Self {
        Self::from_shared(Rc::new(RefCell::new(left)), Rc::new(RefCell::new(right)))
    }

    pub fn from_shared<L: AsMut<[Motor]> + 'static, R: AsMut<[Motor]> + 'static>(
        left: Rc<RefCell<L>>,
        right: Rc<RefCell<R>>,
    ) -> Self {
        Self::from_sides([Side::Shared(left), Side::Shared(right)], None)
    }

    /// Builds geared sides with normalized voltage output, or normalized wheel speed
    /// output when a positive speed limit in meters per second is given.
    pub fn geared(
        left: impl IntoIterator<Item = DriveMotor>,
        right: impl IntoIterator<Item = DriveMotor>,
        wheel_speed_limit: Option<f64>,
    ) -> Option<Self> {
        let left: Vec<_> = left.into_iter().collect();
        let right: Vec<_> = right.into_iter().collect();
        if left.is_empty()
            || right.is_empty()
            || left.len() > crate::feedback::MAX_SIDE_MOTORS
            || right.len() > crate::feedback::MAX_SIDE_MOTORS
            || wheel_speed_limit.is_some_and(|limit| !limit.is_finite() || limit <= 0.0)
        {
            return None;
        }
        Some(Self::from_sides(
            [Side::Geared(left), Side::Geared(right)],
            wheel_speed_limit,
        ))
    }

    /// Builds voltage-controlled geared sides from fixed wiring arrays.
    /// Empty sides and those exceeding encoder consensus capacity fail at compile time.
    pub fn geared_arrays<const L: usize, const R: usize>(
        left: [DriveMotor; L],
        right: [DriveMotor; R],
    ) -> Self {
        const {
            assert!(L > 0 && R > 0, "geared drivetrain requires both sides");
            assert!(
                L <= crate::feedback::MAX_SIDE_MOTORS && R <= crate::feedback::MAX_SIDE_MOTORS,
                "too many geared motors for encoder consensus"
            );
        }
        Self::from_sides(
            [
                Side::Geared(Vec::from(left)),
                Side::Geared(Vec::from(right)),
            ],
            None,
        )
    }

    fn from_sides(sides: [Side; 2], wheel_speed_limit: Option<f64>) -> Self {
        Self {
            sides,
            wheel_speed_limit,
            scale: 1.0,
            slew: None,
            anti_tip: None,
            previous: Instant::now(),
            brake: BrakeMode::Brake,
            feedback: core::array::from_fn(|_| {
                RefCell::new(crate::feedback::EncoderConsensus::default())
            }),
            feedback_time: std::cell::Cell::new(Instant::now()),
        }
    }

    pub fn with_anti_tip(
        mut self,
        imu: Rc<InertialSensor>,
        controller: AntiTip,
        axis: TipAngleAxis,
        level: Angle,
    ) -> Self {
        self.anti_tip = Some(TipControl {
            imu,
            controller,
            axis,
            level,
        });
        self
    }

    pub fn with_slew(mut self, rise_per_second: f64, fall_per_second: f64) -> Option<Self> {
        if ![rise_per_second, fall_per_second]
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

    pub fn wheel_speed_limit(&self) -> Option<f64> {
        self.wheel_speed_limit
    }

    pub fn feedback(&self) -> [Option<WheelFeedback>; 2] {
        let now = Instant::now();
        let dt = now.saturating_duration_since(self.feedback_time.replace(now));
        [0, 1].map(|i| self.sides[i].feedback(&self.feedback[i], dt))
    }

    /// Visit motors in left/right order without sampling or advancing odometry.
    pub fn visit_motors(&self, mut visit: impl FnMut(usize, usize, &Motor)) {
        for (side, motors) in self.sides.iter().enumerate() {
            match motors {
                Side::Geared(motors) => {
                    for (index, motor) in motors.iter().enumerate() {
                        visit(side, index, &motor.motor);
                    }
                }
                Side::Shared(motors) => {
                    for (index, motor) in motors.borrow_mut().as_mut().iter().enumerate() {
                        visit(side, index, motor);
                    }
                }
            }
        }
    }

    /// Read each motor once for load detection without advancing odometry.
    /// A side reports None if any of its motor feedback is unavailable.
    pub fn wheel_load(&self) -> [Option<WheelLoad>; 2] {
        [self.sides[0].wheel_load(), self.sides[1].wheel_load()]
    }

    pub fn measured_voltage(&self) -> [Option<f64>; 2] {
        [
            self.sides[0].voltage_feedback(),
            self.sides[1].voltage_feedback(),
        ]
    }

    /// Stops both sides immediately, bypassing slew and anti-tip.
    pub fn brake_now(&mut self, brake: BrakeMode) -> Result<(), PortError> {
        if let Some(slew) = &mut self.slew {
            slew.iter_mut().for_each(SlewLimiter::reset);
        }
        if let Some(tip) = &mut self.anti_tip {
            tip.controller.reset();
        }
        self.previous = Instant::now();
        each(&mut self.sides, |side| side.stop(brake))
    }

    /// Applies normalized voltage even when tank input is configured for wheel speed.
    pub fn drive_voltage(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        self.apply([left, right], None, false)
    }

    /// Applies volts to every motor on each side, limited by each motor's rating.
    /// Optional output scaling, slew limiting and tilt protection still apply.
    pub fn drive_volts(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        let maximum = Motor::V5_MAX_VOLTAGE;
        self.apply([left / maximum, right / maximum], None, true)
    }

    /// SI wheel-speed control without changing the driver's voltage-control mode.
    /// Requires geared sides. Scaling, slew and anti-tip remain active.
    pub fn drive_wheel_speeds(
        &mut self,
        left: f64,
        right: f64,
        maximum: f64,
    ) -> Result<(), WheelSpeedError> {
        if !maximum.is_finite() || maximum <= 0.0 || !left.is_finite() || !right.is_finite() {
            let _ = self.brake_now(self.brake);
            return Err(WheelSpeedError::InvalidCommand);
        }
        if !self
            .sides
            .iter()
            .all(|side| matches!(side, Side::Geared(_)))
        {
            let _ = self.brake_now(self.brake);
            return Err(WheelSpeedError::NotGeared);
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
        if let Some(tip) = &mut self.anti_tip {
            if let Ok(angles) = tip.imu.euler() {
                let tilt = match tip.axis {
                    TipAngleAxis::Pitch => angles.a,
                    TipAngleAxis::Roll => angles.c,
                };
                let correction = tip
                    .controller
                    .correction((tilt - tip.level).wrapped_half().as_radians(), dt);
                command = desaturate(command.map(|power| power + correction), 1.0);
            } else {
                tip.controller.reset();
            }
        }
        if let Some(slew) = &mut self.slew {
            command = [
                slew[0].update(command[0], dt),
                slew[1].update(command[1], dt),
            ];
        }
        let [left, right] = &mut self.sides;
        let [left_power, right_power] = command.map(|power| power * self.scale);
        match limit {
            Some(limit) => left
                .speed(left_power * limit)
                .and(right.speed(right_power * limit)),
            None => left
                .voltage(left_power, absolute)
                .and(right.voltage(right_power, absolute)),
        }
    }
}

impl DrivetrainModel for Differential {
    type Error = PortError;
}

impl Tank for Differential {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        self.apply([left, right], self.wheel_speed_limit, false)
    }

    fn stop_now(&mut self) -> Result<(), PortError> {
        self.brake_now(self.brake)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn physical_voltage_preserves_setpoint_for_mixed_motors() {
        assert_eq!(motor_voltage(0.5, 8.0, true), 6.0);
        assert_eq!(motor_voltage(0.5, 12.0, true), 6.0);
        assert_eq!(motor_voltage(-1.0, 8.0, true), -8.0);
        assert_eq!(motor_voltage(0.5, 8.0, false), 4.0);
    }
}
