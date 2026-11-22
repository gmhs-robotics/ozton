//! Physical wheel measurements and calibrated conversions.
use ozton_math::uom::si::{
    angular_velocity::revolution_per_minute,
    electric_current::ampere,
    f64::{AngularVelocity, ElectricCurrent, Length, Velocity},
    length::meter,
    velocity::meter_per_second,
};
use vexide::{
    prelude::Motor,
    smart::{PortError, motor::BrakeMode},
};

use crate::feedback::{EncoderConsensus, MAX_SIDE_MOTORS};

/// Motor cartridge output turns to wheel turns. Supply each stage's driver and driven tooth counts.
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DriveConfigError {
    WheelDiameter,
    WheelGearing,
    FreeWheelSpeed,
    TrackWidth,
    Wheelbase,
}

impl std::fmt::Display for DriveConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::WheelDiameter => "wheel diameter must be finite and positive",
            Self::WheelGearing => "supply a finite positive ratio for every motor in each group",
            Self::FreeWheelSpeed => "free wheel speed must be finite and positive",
            Self::TrackWidth => "track width must be finite and positive",
            Self::Wheelbase => "wheelbase must be finite and positive",
        })
    }
}

impl std::error::Error for DriveConfigError {}

pub(crate) fn positive(value: f64, error: DriveConfigError) -> Result<(), DriveConfigError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(error)
    }
}

enum Gearing {
    Explicit(Box<[Box<[WheelGearing]>]>),
    FreeRpm(f64),
}

pub(crate) struct Wheels {
    diameter: Length,
    circumference: f64,
    gearing: Gearing,
    groups: Box<[EncoderConsensus]>,
}

impl Wheels {
    pub(crate) fn geared(
        diameter: Length,
        motors: &[Box<[Motor]>],
        ratios: &[&[WheelGearing]],
    ) -> Result<Self, DriveConfigError> {
        if ratios.len() != motors.len()
            || ratios.iter().zip(motors).any(|(ratios, motors)| {
                ratios.len() != motors.len()
                    || ratios
                        .iter()
                        .any(|gear| !gear.0.is_finite() || gear.0 <= 0.0)
            })
        {
            return Err(DriveConfigError::WheelGearing);
        }
        Self::new(
            diameter,
            motors,
            Gearing::Explicit(ratios.iter().map(|ratios| (*ratios).into()).collect()),
        )
    }

    pub(crate) fn inferred(
        diameter: Length,
        motors: &[Box<[Motor]>],
        speed: AngularVelocity,
    ) -> Result<Self, DriveConfigError> {
        let rpm = speed.get::<revolution_per_minute>();
        positive(rpm, DriveConfigError::FreeWheelSpeed)?;
        Self::new(diameter, motors, Gearing::FreeRpm(rpm))
    }

    fn new(
        diameter: Length,
        motors: &[Box<[Motor]>],
        gearing: Gearing,
    ) -> Result<Self, DriveConfigError> {
        let circumference = diameter.value * std::f64::consts::PI;
        positive(circumference, DriveConfigError::WheelDiameter)?;
        if motors.is_empty()
            || motors
                .iter()
                .any(|motors| motors.is_empty() || motors.len() > MAX_SIDE_MOTORS)
        {
            return Err(DriveConfigError::WheelGearing);
        }
        Ok(Self {
            diameter,
            circumference,
            gearing,
            groups: motors.iter().map(|_| EncoderConsensus::default()).collect(),
        })
    }

    pub(crate) fn diameter(&self) -> Length {
        self.diameter
    }

    fn meters_per_turn(&self, group: usize, index: usize, motor: &Motor) -> Result<f64, PortError> {
        Ok(self.circumference
            * match &self.gearing {
                Gearing::Explicit(ratios) => ratios[group][index].0,
                Gearing::FreeRpm(rpm) => rpm / motor.gearset()?.max_rpm(),
            })
    }

    fn group_matches(&self, group: usize, motors: &[Motor]) -> bool {
        !motors.is_empty()
            && motors.len() <= MAX_SIDE_MOTORS
            && match &self.gearing {
                Gearing::Explicit(ratios) => ratios
                    .get(group)
                    .is_some_and(|ratios| ratios.len() == motors.len()),
                Gearing::FreeRpm(_) => true,
            }
    }

    pub(crate) fn drive_speed<const N: usize>(
        &self,
        motors: &mut [Box<[Motor]>],
        speeds: [Velocity; N],
    ) -> Result<(), WheelSpeedError> {
        if N != motors.len()
            || N != self.groups.len()
            || speeds.iter().any(|speed| !speed.value.is_finite())
            || motors
                .iter()
                .enumerate()
                .any(|(group, motors)| !self.group_matches(group, motors))
        {
            return Err(WheelSpeedError::InvalidCommand);
        }
        let mut result = Ok(());
        for (group, (motors, speed)) in motors.iter_mut().zip(speeds).enumerate() {
            for (index, motor) in motors.iter_mut().enumerate() {
                let written = if speed.value == 0.0 {
                    motor.brake(BrakeMode::Coast)
                } else {
                    self.meters_per_turn(group, index, motor)
                        .and_then(|distance| {
                            motor.set_velocity((speed.value * 60.0 / distance).round() as i32)
                        })
                };
                result = result.and(written);
            }
        }
        result.map_err(WheelSpeedError::Port)
    }

    pub(crate) fn sample<const N: usize>(
        &mut self,
        motors: &[Box<[Motor]>],
    ) -> [Option<WheelFeedback>; N] {
        core::array::from_fn(|group| {
            let motors = motors.get(group)?;
            if !self.group_matches(group, motors) {
                return None;
            }
            let mut samples = [None; MAX_SIDE_MOTORS];
            for (index, motor) in motors.iter().enumerate() {
                samples[index] = (|| {
                    let distance = self.meters_per_turn(group, index, motor).ok()?;
                    Some(WheelFeedback {
                        position: Length::new::<meter>(
                            motor.position().ok()?.as_turns() * distance,
                        ),
                        velocity: Velocity::new::<meter_per_second>(
                            motor.velocity().ok()? * distance / 60.0,
                        ),
                    })
                })();
            }
            self.groups.get_mut(group)?.sample(&samples[..motors.len()])
        })
    }

    pub(crate) fn load(&self, group: usize, motors: &[Motor]) -> Option<WheelLoad> {
        if !self.group_matches(group, motors) {
            return None;
        }
        let mut current = 0.0;
        let mut fastest: f64 = 0.0;
        for (index, motor) in motors.iter().enumerate() {
            let amps = motor.current().ok()?;
            let speed =
                motor.velocity().ok()? * self.meters_per_turn(group, index, motor).ok()? / 60.0;
            if !amps.is_finite() || !speed.is_finite() {
                return None;
            }
            current += amps.abs();
            fastest = fastest.max(speed.abs());
        }
        Some(WheelLoad {
            mean_current: ElectricCurrent::new::<ampere>(current / motors.len() as f64),
            fastest_wheel_speed: Velocity::new::<meter_per_second>(fastest),
        })
    }
}

/// Continuous group travel and velocity from all available motors.
#[derive(Clone, Copy, Debug)]
pub struct WheelFeedback {
    pub position: Length,
    pub velocity: Velocity,
}

/// Motor load and maximum absolute wheel speed across a group.
#[derive(Clone, Copy, Debug)]
pub struct WheelLoad {
    pub mean_current: ElectricCurrent,
    pub fastest_wheel_speed: Velocity,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WheelSpeedError {
    InvalidCommand,
    MissingWheelGeometry,
    Port(PortError),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compound_gearing_matches_blue_wheel_speed() {
        assert_eq!(600.0 * WheelGearing::new(36, 48).0, 450.0);
        assert_eq!(200.0 * WheelGearing::new(36, 12).then(36, 48).0, 450.0);
    }
}
