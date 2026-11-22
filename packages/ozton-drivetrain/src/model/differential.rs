//! Differential voltage output and optional wheel-speed feedback.

use std::time::Instant;

use ozton_control::drive::SlewLimiter;
use ozton_math::uom::si::{
    electric_potential::volt,
    f64::{AngularVelocity, ElectricPotential, Frequency, Length, Velocity},
};
use vexide::{
    prelude::Motor,
    smart::{PortError, motor::BrakeMode},
};

use super::{
    Arcade, ArcadeInput, DrivetrainModel, ProgramDrive, Tank,
    driver::DriverControl,
    wheels::{
        DriveMotor, WheelFeedback, WheelGeometry, WheelLoad, WheelSpeedError, each, shape,
        valid_gearing, write_wheels,
    },
};

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
    wheelbase: Option<Length>,
    wheel_speed_limit: Option<Velocity>,
    scale: f64,
    driver: DriverControl,
    maximum_wheel_speed: Option<Velocity>,
    output: ([f64; 2], f64),
    status: ozton_diagnostics::serial::FaultLog,
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
    pub fn new<L: Into<DriveMotor>, R: Into<DriveMotor>>(
        left: impl IntoIterator<Item = L>,
        right: impl IntoIterator<Item = R>,
    ) -> Self {
        Self::from_sides(
            [
                left.into_iter().map(Into::into).collect(),
                right.into_iter().map(Into::into).collect(),
            ],
            None,
        )
    }

    /// Set wheel diameter when every motor has explicit gearing.
    pub fn with_wheel_diameter(mut self, diameter: Length) -> Option<Self> {
        let circumference = diameter * core::f64::consts::PI;
        if !circumference.value.is_finite()
            || circumference.value <= 0.0
            || !self.sides.iter().all(|side| valid_gearing(side))
        {
            return None;
        }
        self.wheels = Some(WheelGeometry {
            free_rpm: None,
            circumference,
        });
        Some(self)
    }

    fn from_sides(sides: [Box<[DriveMotor]>; 2], wheels: Option<WheelGeometry>) -> Self {
        Self {
            sides,
            wheels,
            track_width: None,
            wheelbase: None,
            wheel_speed_limit: None,
            scale: 1.0,
            driver: DriverControl::default(),
            maximum_wheel_speed: None,
            output: ([0.0; 2], 1.0),
            status: ozton_diagnostics::serial::FaultLog::new("drive", vexide::time::user_uptime),
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
        self.wheels = Some(WheelGeometry::new(diameter, Some(free_speed))?);
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

    pub fn with_wheelbase(mut self, wheelbase: Length) -> Option<Self> {
        if !wheelbase.value.is_finite() || wheelbase.value <= 0.0 {
            return None;
        }
        self.wheelbase = Some(wheelbase);
        Some(self)
    }

    pub fn wheelbase(&self) -> Option<Length> {
        self.wheelbase
    }

    /// Limit programmed wheel-speed commands. Driver control still uses voltage.
    pub fn with_maximum_wheel_speed(mut self, speed: Velocity) -> Option<Self> {
        if self.wheels.is_none() || !speed.value.is_finite() || speed.value <= 0.0 {
            return None;
        }
        self.maximum_wheel_speed = Some(speed);
        Some(self)
    }

    pub fn maximum_wheel_speed(&self) -> Option<Velocity> {
        self.maximum_wheel_speed.or(self.wheel_speed_limit)
    }

    pub fn wheel_diameter(&self) -> Option<Length> {
        self.wheels
            .map(|wheels| wheels.circumference / core::f64::consts::PI)
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
        self.driver.reset();
        self.output = ([0.0; 2], self.scale);
        let result = each(&mut self.sides, |side| {
            each(side, |drive| drive.motor.brake(brake))
        });
        self.record_output(&result);
        result
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
    /// Requires wheel calibration. Output scaling and slew remain active.
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
        let command = shape(input, &mut self.slew, &mut self.previous);
        self.output = (command, self.scale);
        let result = write_wheels(
            &mut self.sides,
            command.map(|power| power * self.scale),
            self.wheels.zip(limit),
            absolute,
        );
        self.record_output(&result);
        result
    }
    fn record_output(&mut self, result: &Result<(), PortError>) {
        match result {
            Ok(()) => self.status.recovered(),
            Err(error) => self
                .status
                .fail("write_failed", format_args!(",error={error:?}")),
        }
    }

    #[cfg(any(debug_assertions, feature = "sd-logging"))]
    pub fn snapshot(&self) -> super::DriveSnapshot {
        let meters = self
            .last_feedback
            .map(|sample| sample.map_or(f64::NAN, |v| v.position.value));
        let circumference = self.wheels.map_or(f64::NAN, |w| w.circumference.value);
        super::DriveSnapshot {
            command: self.output.0,
            scale: self.output.1,
            wheel_meters: meters,
            wheel_revolutions: meters.map(|v| v / circumference),
            write_failed: self.status.code().is_some(),
            fault_count: self.status.count(),
        }
    }

    super::driver::driver_builders!();

    /// Apply driver assistance and the measured-load cap before mixing the wheels.
    pub fn drive(&mut self, input: ArcadeInput) -> Result<(), PortError> {
        let (forward, turn, scale) = self.driver.apply(
            input,
            std::time::Instant::now().saturating_duration_since(self.previous),
            0.0,
        );
        let base = self.scale;
        self.scale = scale * base;
        let result = self.drive_tank(forward + turn, forward - turn);
        self.scale = base;
        result
    }

    pub fn stop(&mut self) -> Result<(), PortError> {
        self.brake_now(self.brake)
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

impl ProgramDrive for Differential {
    type Error = WheelSpeedError;
    type DriverCommand = ArcadeInput;
    type AutonCommand = [Velocity; 2];

    fn apply_driver(
        &mut self,
        command: ArcadeInput,
        _: std::time::Duration,
    ) -> Result<(), Self::Error> {
        self.drive(command).map_err(WheelSpeedError::Port)
    }

    fn apply_auton(
        &mut self,
        [left, right]: Self::AutonCommand,
        _: std::time::Duration,
    ) -> Result<(), Self::Error> {
        let Some(limit) = self.maximum_wheel_speed() else {
            let _ = self.stop();
            return Err(WheelSpeedError::MissingWheelGeometry);
        };
        self.drive_wheel_speeds(left, right, limit)
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        Differential::stop(self).map_err(WheelSpeedError::Port)
    }
}
