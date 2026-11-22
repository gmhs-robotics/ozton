use std::time::{Duration, Instant};

use glam::DVec2 as Vec2;
use ozton_control::drive::SlewLimiter;
use ozton_math::uom::si::f64::{AngularVelocity, Frequency, Length, Velocity};
use vexide::smart::{PortError, motor::BrakeMode};

use super::{
    ArcadeInput, DrivetrainModel, Holonomic, ProgramDrive, Tank,
    driver::DriverControl,
    wheels::{
        DriveConfigError, DriveMotor, MaxSpeed, WheelFeedback, WheelGeometry, WheelLoad,
        WheelOutput, WheelSpeedError, each, positive, shape, write_wheels,
    },
};

/// Four mecanum motor groups in front-left, front-right, back-left, back-right order.
pub struct Mecanum {
    motors: [Box<[DriveMotor]>; 4],
    driver: DriverControl,
    wheels: Option<WheelGeometry>,
    max_speed: MaxSpeed,
    auton_speed_limit: Option<Velocity>,
    slew: Option<[SlewLimiter; 4]>,
    scale: f64,
    brake: BrakeMode,
    previous: Instant,
    feedback_time: Instant,
    consensus: [crate::feedback::EncoderConsensus; 4],
}

impl Mecanum {
    pub fn new<M: Into<DriveMotor>>(
        front_left: impl IntoIterator<Item = M>,
        front_right: impl IntoIterator<Item = M>,
        back_left: impl IntoIterator<Item = M>,
        back_right: impl IntoIterator<Item = M>,
    ) -> Self {
        Self {
            motors: [
                front_left.into_iter().map(Into::into).collect(),
                front_right.into_iter().map(Into::into).collect(),
                back_left.into_iter().map(Into::into).collect(),
                back_right.into_iter().map(Into::into).collect(),
            ],
            driver: DriverControl::default(),
            wheels: None,
            max_speed: MaxSpeed::Fraction(1.0),
            auton_speed_limit: None,
            slew: None,
            scale: 1.0,
            brake: BrakeMode::Brake,
            previous: Instant::now(),
            feedback_time: Instant::now(),
            consensus: core::array::from_fn(|_| Default::default()),
        }
    }

    super::driver::driver_builders!();

    pub fn with_wheel_diameter(mut self, diameter: Length) -> Result<Self, DriveConfigError> {
        self.wheels = Some(WheelGeometry::new(diameter, None, &self.motors)?);
        Ok(self)
    }

    pub fn with_wheel_feedback(
        mut self,
        diameter: Length,
        free_speed: AngularVelocity,
    ) -> Result<Self, DriveConfigError> {
        self.wheels = Some(WheelGeometry::new(
            diameter,
            Some(free_speed),
            &self.motors,
        )?);
        Ok(self)
    }

    /// Set the voltage ceiling for drive commands. Configure wheel geometry first for `Wheel`.
    pub fn with_max_speed(mut self, speed: MaxSpeed) -> Result<Self, DriveConfigError> {
        speed.validate(self.wheels)?;
        self.max_speed = speed;
        Ok(self)
    }

    /// Set the wheel velocity ceiling for native autonomous speed commands.
    pub fn with_auton_speed_limit(mut self, speed: Velocity) -> Result<Self, DriveConfigError> {
        if self.wheels.is_none() {
            return Err(DriveConfigError::MissingWheelGeometry);
        }
        positive(speed.value, DriveConfigError::AutonSpeedLimit)?;
        self.auton_speed_limit = Some(speed);
        Ok(self)
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

    pub fn set_output_scale(&mut self, scale: f64) {
        self.scale = if scale.is_finite() {
            scale.clamp(0.0, 1.0)
        } else {
            0.0
        };
    }

    pub fn wheel_diameter(&self) -> Option<Length> {
        self.wheels
            .map(|wheels| wheels.circumference / core::f64::consts::PI)
    }

    /// Native autonomous wheel velocity ceiling, if configured.
    pub fn auton_speed_limit(&self) -> Option<Velocity> {
        self.auton_speed_limit.or(match self.max_speed {
            MaxSpeed::Wheel(speed) if speed.value > 0.0 => Some(speed),
            _ => None,
        })
    }

    pub fn sample_feedback(&mut self) -> [Option<WheelFeedback>; 4] {
        let dt = Instant::now().saturating_duration_since(self.feedback_time);
        self.feedback_time = Instant::now();
        core::array::from_fn(|i| {
            self.wheels?
                .feedback(&self.motors[i], &mut self.consensus[i], dt)
        })
    }

    pub fn wheel_load(&self) -> [Option<WheelLoad>; 4] {
        self.motors
            .each_ref()
            .map(|motors| self.wheels?.wheel_load(motors))
    }

    /// Left stick strafe and forward plus turn. Heading and tilt come from the tracking sample.
    pub fn drive(&mut self, strafe: f64, input: ArcadeInput) -> Result<(), PortError> {
        let (forward, turn) = self.driver.apply(input, self.previous.elapsed(), strafe);
        self.drive_wheels(
            mix(strafe, forward, turn),
            input.output_scale,
            self.voltage_output(),
        )
    }

    fn voltage_output(&self) -> WheelOutput {
        WheelOutput::Voltage {
            limit: self.max_speed,
            absolute: false,
        }
    }

    fn drive_wheels(
        &mut self,
        powers: [f64; 4],
        scale: f64,
        output: WheelOutput,
    ) -> Result<(), PortError> {
        if powers.iter().any(|power| !power.is_finite()) {
            return self.stop_now();
        }
        let powers = shape(powers, &mut self.slew, &mut self.previous);
        write_wheels(
            &mut self.motors,
            powers.map(|power| power * scale * self.scale),
            self.wheels,
            output,
        )
    }
}

fn mix(strafe: f64, forward: f64, turn: f64) -> [f64; 4] {
    [
        forward + strafe + turn,
        forward - strafe - turn,
        forward - strafe + turn,
        forward + strafe - turn,
    ]
}

impl DrivetrainModel for Mecanum {
    type Error = PortError;
}

impl Holonomic for Mecanum {
    fn drive_vector(&mut self, vector: Vec2, turn: f64) -> Result<(), PortError> {
        self.drive_wheels(mix(vector.x, vector.y, turn), 1.0, self.voltage_output())
    }
}

impl Tank for Mecanum {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), PortError> {
        self.drive_wheels([left, right, left, right], 1.0, self.voltage_output())
    }

    fn stop_now(&mut self) -> Result<(), PortError> {
        self.driver.reset();
        self.previous = Instant::now();
        if let Some(slew) = &mut self.slew {
            slew.iter_mut().for_each(SlewLimiter::reset);
        }
        each(&mut self.motors, |motors| {
            each(motors, |drive| drive.motor.brake(self.brake))
        })
    }
}

impl ProgramDrive for Mecanum {
    type Error = WheelSpeedError;
    type DriverCommand = (f64, ArcadeInput);
    type AutonCommand = [Velocity; 4];

    fn apply_driver(
        &mut self,
        (strafe, input): Self::DriverCommand,
        _: Duration,
    ) -> Result<(), Self::Error> {
        self.drive(strafe, input).map_err(WheelSpeedError::Port)
    }

    fn apply_auton(&mut self, speeds: Self::AutonCommand, _: Duration) -> Result<(), Self::Error> {
        let Some(limit) = self.auton_speed_limit() else {
            let _ = self.stop_now();
            return Err(WheelSpeedError::MissingWheelGeometry);
        };
        if speeds.iter().any(|speed| !speed.value.is_finite()) {
            let _ = self.stop_now();
            return Err(WheelSpeedError::InvalidCommand);
        }
        self.drive_wheels(
            speeds.map(|speed| speed.value / limit.value),
            1.0,
            WheelOutput::Velocity(limit.value),
        )
        .map_err(WheelSpeedError::Port)
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.stop_now().map_err(WheelSpeedError::Port)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wheel_order_supports_strafe_and_turn() {
        assert_eq!(mix(1.0, 0.0, 0.0), [1.0, -1.0, -1.0, 1.0]);
        assert_eq!(mix(0.0, 0.0, 1.0), [1.0, -1.0, 1.0, -1.0]);
    }
}
