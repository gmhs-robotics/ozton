//! External gearing and per-motor wheel conversions.

use core::f64::consts::PI;

use ozton_math::units::Length;
use vexide::prelude::Motor;

/// Wheel turns per motor output turn. Motor direction handles gear polarity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GearRatio(f64);

impl GearRatio {
    pub const DIRECT: Self = Self(1.0);

    pub const fn from_teeth(driver: u32, driven: u32) -> Self {
        assert!(driver > 0 && driven > 0);
        Self(driver as f64 / driven as f64)
    }

    pub const fn new(wheel_per_motor: f64) -> Option<Self> {
        if wheel_per_motor.is_finite() && wheel_per_motor > 0.0 {
            Some(Self(wheel_per_motor))
        } else {
            None
        }
    }

    pub const fn then(self, stage: Self) -> Self {
        let ratio = self.0 * stage.0;
        assert!(ratio.is_finite() && ratio > 0.0);
        Self(ratio)
    }

    pub const fn wheel_per_motor(self) -> f64 {
        self.0
    }
}

/// Precomputed wheel conversions for one motor's gearing and wheel diameter.
#[derive(Debug, Clone, Copy)]
pub struct DriveMotorCalibration {
    meters_per_turn: f64,
    rpm_per_meter_per_second: f64,
}

impl DriveMotorCalibration {
    /// Validates wheel conversions. Use in a constant to check static drivetrain settings.
    pub const fn new(ratio: GearRatio, wheel_diameter: Length) -> Option<Self> {
        let meters_per_turn = ratio.0 * wheel_diameter.as_meters() * PI;
        let rpm_per_meter_per_second = 60.0 / meters_per_turn;
        if !meters_per_turn.is_finite()
            || meters_per_turn <= 0.0
            || !rpm_per_meter_per_second.is_finite()
        {
            return None;
        }
        Some(Self {
            meters_per_turn,
            rpm_per_meter_per_second,
        })
    }
}

/// A motor and the effective diameter of the wheels it drives.
pub struct DriveMotor {
    pub(crate) motor: Motor,
    pub(crate) meters_per_turn: f64,
    pub(crate) rpm_per_meter_per_second: f64,
}

impl DriveMotor {
    pub fn new(motor: Motor, ratio: GearRatio, wheel_diameter: f64) -> Option<Self> {
        Some(Self::calibrated(
            motor,
            DriveMotorCalibration::new(ratio, Length::meters(wheel_diameter))?,
        ))
    }

    /// Attach a motor to previously validated wheel conversions.
    pub const fn calibrated(motor: Motor, calibration: DriveMotorCalibration) -> Self {
        Self {
            motor,
            meters_per_turn: calibration.meters_per_turn,
            rpm_per_meter_per_second: calibration.rpm_per_meter_per_second,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn idler_cancels_in_compound_ratio() {
        const RATIO: GearRatio = GearRatio::from_teeth(36, 24).then(GearRatio::from_teeth(24, 48));
        assert_eq!(RATIO.wheel_per_motor(), 0.75);
    }
}
