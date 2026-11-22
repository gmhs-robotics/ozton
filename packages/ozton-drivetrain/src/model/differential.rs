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
    ArcadeInput, DrivetrainModel, ProgramDrive, Tank,
    driver::DriverControl,
    wheels::{
        DriveConfigError, DriveMotor, MaxSpeed, WheelFeedback, WheelGeometry, WheelLoad,
        WheelSpeedError, each, positive, shape, write_voltage, write_wheel_speeds,
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
    driver: DriverControl,
    driver_max_speed: MaxSpeed,
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
    pub fn with_wheel_diameter(mut self, diameter: Length) -> Result<Self, DriveConfigError> {
        self.wheels = Some(WheelGeometry::new(diameter, None, &self.sides)?);
        Ok(self)
    }

    fn from_sides(sides: [Box<[DriveMotor]>; 2], wheels: Option<WheelGeometry>) -> Self {
        Self {
            sides,
            wheels,
            track_width: None,
            wheelbase: None,
            driver: DriverControl::default(),
            driver_max_speed: MaxSpeed::Fraction(1.0),
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
    ) -> Result<Self, DriveConfigError> {
        self.wheels = Some(WheelGeometry::new(diameter, Some(free_speed), &self.sides)?);
        Ok(self)
    }

    /// Set the distance between the left and right drive contact lines.
    pub fn with_track_width(mut self, width: Length) -> Result<Self, DriveConfigError> {
        positive(width.value, DriveConfigError::TrackWidth)?;
        self.track_width = Some(width);
        Ok(self)
    }

    /// Calibrated distance between the left and right drive contact lines.
    pub fn track_width(&self) -> Option<Length> {
        self.track_width
    }

    pub fn with_wheelbase(mut self, wheelbase: Length) -> Result<Self, DriveConfigError> {
        positive(wheelbase.value, DriveConfigError::Wheelbase)?;
        self.wheelbase = Some(wheelbase);
        Ok(self)
    }

    pub fn wheelbase(&self) -> Option<Length> {
        self.wheelbase
    }

    /// Set the voltage ceiling used by the assisted driver input.
    /// Configure wheel geometry first when using `MaxSpeed::Wheel`.
    pub fn max_speed(&mut self, speed: MaxSpeed) -> Result<(), DriveConfigError> {
        speed.validate(self.wheels)?;
        self.driver_max_speed = speed;
        Ok(())
    }

    pub fn wheel_diameter(&self) -> Option<Length> {
        self.wheels
            .map(|wheels| wheels.circumference / core::f64::consts::PI)
    }

    pub fn with_slew(mut self, rise: Frequency, fall: Frequency) -> Result<Self, DriveConfigError> {
        positive(rise.value, DriveConfigError::SlewRate)?;
        positive(fall.value, DriveConfigError::SlewRate)?;
        self.slew = Some(core::array::from_fn(|_| SlewLimiter::new(rise, fall)));
        Ok(self)
    }

    pub fn with_brake(mut self, brake: BrakeMode) -> Self {
        self.brake = brake;
        self
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
        self.output = ([0.0; 2], 1.0);
        let result = each(&mut self.sides, |side| {
            each(side, |drive| drive.motor.brake(brake))
        });
        self.record_output(&result);
        result
    }

    /// Apply normalized voltage using each motor's rating.
    pub fn drive_normalized_voltage(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        self.write_voltage(
            ozton_math::desaturate([left, right], 1.0),
            1.0,
            MaxSpeed::Fraction(1.0),
            false,
        )
    }

    /// Apply volts to every motor on each side, bounded by its rating.
    pub fn drive_volts(
        &mut self,
        left: ElectricPotential,
        right: ElectricPotential,
    ) -> Result<(), PortError> {
        let maximum = Motor::V5_MAX_VOLTAGE;
        self.write_voltage(
            ozton_math::desaturate([left.value / maximum, right.value / maximum], 1.0),
            1.0,
            MaxSpeed::Fraction(1.0),
            true,
        )
    }

    /// Command native wheel velocity using the calibrated gearing and wheel diameter.
    pub fn drive_wheel_speeds(
        &mut self,
        left: Velocity,
        right: Velocity,
    ) -> Result<(), WheelSpeedError> {
        match write_wheel_speeds(&mut self.sides, [left, right], self.wheels) {
            Ok(()) => {
                self.output = (ozton_math::desaturate([left.value, right.value], 1.0), 1.0);
                self.record_output(&Ok(()));
                Ok(())
            }
            Err(WheelSpeedError::Port(port)) => {
                self.record_output(&Err(port));
                Err(WheelSpeedError::Port(port))
            }
            Err(error) => {
                let _ = self.stop();
                Err(error)
            }
        }
    }

    pub fn drive_driver_arcade(
        &mut self,
        throttle: f64,
        steer: f64,
        policy: DriverFaultPolicy,
    ) -> Result<(), PortError> {
        let output = self.drive(ArcadeInput::new(throttle, steer));
        if output.is_err() && matches!(policy, DriverFaultPolicy::Stop) {
            let _ = self.stop_now();
        }
        output
    }

    fn write_voltage(
        &mut self,
        input: [f64; 2],
        scale: f64,
        limit: MaxSpeed,
        absolute: bool,
    ) -> Result<(), PortError> {
        if input.iter().any(|value| !value.is_finite()) {
            return self.stop_now();
        }
        self.output = (input, scale);
        let result = write_voltage(
            &mut self.sides,
            input.map(|power| power * scale),
            self.wheels,
            limit,
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

    /// Mix arcade input with the configured driver assists.
    pub fn drive(&mut self, input: ArcadeInput) -> Result<(), PortError> {
        let (forward, turn) = self.driver.apply(input, self.previous.elapsed(), 0.0);
        let powers = [forward + turn, forward - turn];
        if powers.iter().any(|power| !power.is_finite()) {
            return self.stop_now();
        }
        let powers = shape(powers, &mut self.slew, &mut self.previous);
        self.write_voltage(powers, input.output_scale, self.driver_max_speed, false)
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
        self.drive_normalized_voltage(left, right)
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
        self.drive_wheel_speeds(left, right)
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        Differential::stop(self).map_err(WheelSpeedError::Port)
    }
}
