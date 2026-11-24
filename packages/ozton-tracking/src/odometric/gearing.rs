//! Physical wheel measurements and calibrated conversions.
use ozton_math::uom::si::{
    angular_velocity::revolution_per_minute,
    f64::{AngularVelocity, Length, Velocity},
    length::meter,
    velocity::meter_per_second,
};
use vexide::{prelude::Motor, smart::PortError};

use super::{
    SharedMotors, WheelFeedback,
    consensus::{EncoderConsensus, MAX_SIDE_MOTORS},
};

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
}

impl std::fmt::Display for DriveConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::WheelDiameter => "wheel diameter must be finite and positive",
            Self::WheelGearing => "supply a finite positive ratio for every motor in each group",
            Self::FreeWheelSpeed => "free wheel speed must be finite and positive",
            Self::TrackWidth => "track width must be finite and positive",
        })
    }
}

impl std::error::Error for DriveConfigError {}

pub fn positive(value: f64, error: DriveConfigError) -> Result<(), DriveConfigError> {
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

pub struct Wheels {
    diameter: Length,
    circumference: f64,
    gearing: Gearing,
    groups: Box<[EncoderConsensus]>,
}

impl Wheels {
    pub fn geared(
        diameter: Length,
        motors: &[SharedMotors],
        ratios: &[&[WheelGearing]],
    ) -> Result<Self, DriveConfigError> {
        if ratios.len() != motors.len()
            || ratios.iter().zip(motors).any(|(ratios, motors)| {
                ratios.len() != motors.borrow().len()
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

    pub fn inferred(
        diameter: Length,
        motors: &[SharedMotors],
        speed: AngularVelocity,
    ) -> Result<Self, DriveConfigError> {
        let rpm = speed.get::<revolution_per_minute>();
        positive(rpm, DriveConfigError::FreeWheelSpeed)?;
        Self::new(diameter, motors, Gearing::FreeRpm(rpm))
    }

    fn new(
        diameter: Length,
        motors: &[SharedMotors],
        gearing: Gearing,
    ) -> Result<Self, DriveConfigError> {
        let circumference = diameter.value * std::f64::consts::PI;
        positive(circumference, DriveConfigError::WheelDiameter)?;
        if motors.is_empty()
            || motors.iter().any(|motors| {
                let motors = motors.borrow();
                motors.is_empty() || motors.len() > MAX_SIDE_MOTORS
            })
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

    pub fn diameter(&self) -> Length {
        self.diameter
    }

    pub fn meters_per_turn(
        &self,
        group: usize,
        index: usize,
        motor: &Motor,
    ) -> Result<f64, PortError> {
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

    pub fn matches(&self, groups: &[SharedMotors]) -> bool {
        groups.len() == self.groups.len()
            && groups
                .iter()
                .enumerate()
                .all(|(index, group)| self.group_matches(index, &group.borrow()))
    }

    pub fn sample<const N: usize>(
        &mut self,
        motors: &[SharedMotors],
    ) -> [Option<WheelFeedback>; N] {
        core::array::from_fn(|group| {
            let motors = motors.get(group)?.borrow();
            if !self.group_matches(group, &motors) {
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
