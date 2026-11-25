//! Hardware-independent motor stopping and mixed wheel commands.

use ozton_math::uom::si::f64::Velocity;

/// How an actuator behaves when explicitly stopped.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BrakeMode {
    /// Release motor torque.
    #[default]
    Coast,
    /// Electrically resist motion without holding a position.
    Brake,
    /// Actively maintain the stopping position.
    Hold,
}

/// An actuator that supports explicit stopping modes, independent of its vendor.
pub trait HasBraking {
    type BrakeError;

    fn apply_brake(&mut self, mode: BrakeMode) -> Result<(), Self::BrakeError>;
}

/// A motor's location in the model's fixed wheel-group and motor order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MotorAddress {
    pub group: usize,
    pub motor: usize,
}

/// Selective stopping, independent of the motor vendor and drive geometry.
pub trait BrakeMotors: HasBraking {
    /// Apply one stopping mode per selected motor. None leaves its output unchanged.
    /// Attempt every selected motor even if an earlier operation fails.
    fn brake_motors(
        &mut self,
        select: impl FnMut(MotorAddress) -> Option<BrakeMode>,
    ) -> Result<(), Self::BrakeError>;

    fn brake_all(&mut self, mode: BrakeMode) -> Result<(), Self::BrakeError> {
        self.apply_brake(mode)
    }
}

/// Velocity and explicit stopping are distinct actuator commands.
/// A zero velocity command is not an active position hold.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WheelCommand {
    Velocity(Velocity),
    Brake(BrakeMode),
}

impl WheelCommand {
    pub fn velocity(self) -> Velocity {
        match self {
            Self::Velocity(speed) => speed,
            Self::Brake(_) => ozton_math::qty!(0 m/s),
        }
    }

    pub fn is_valid(self) -> bool {
        self.velocity().value.is_finite()
    }
}

/// Two-sided physical wheel actuation with independent braking.
/// No localization or competition executor is required to implement this capability.
pub trait WheelActuation: HasBraking {
    type WheelError;

    fn drive_wheel_commands(&mut self, commands: [WheelCommand; 2])
    -> Result<(), Self::WheelError>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn selective_modes_and_mixed_commands_preserve_intent() {
        struct Mock([Option<BrakeMode>; 3]);
        impl HasBraking for Mock {
            type BrakeError = core::convert::Infallible;

            fn apply_brake(&mut self, mode: BrakeMode) -> Result<(), Self::BrakeError> {
                self.brake_motors(|_| Some(mode))
            }
        }

        impl BrakeMotors for Mock {
            fn brake_motors(
                &mut self,
                mut select: impl FnMut(MotorAddress) -> Option<BrakeMode>,
            ) -> Result<(), Self::BrakeError> {
                for (motor, previous) in self.0.iter_mut().enumerate() {
                    if let Some(mode) = select(MotorAddress { group: 0, motor }) {
                        *previous = Some(mode);
                    }
                }
                Ok(())
            }
        }
        let mut mock = Mock([None; 3]);
        mock.brake_motors(|address| match address.motor {
            0 => Some(BrakeMode::Hold),
            1 => Some(BrakeMode::Coast),
            _ => None,
        })
        .unwrap();
        assert_eq!(
            mock.0,
            [Some(BrakeMode::Hold), Some(BrakeMode::Coast), None]
        );
        mock.brake_all(BrakeMode::Brake).unwrap();
        assert_eq!(mock.0, [Some(BrakeMode::Brake); 3]);
        let hold = WheelCommand::Brake(BrakeMode::Hold);
        assert_eq!(hold.velocity().value, 0.0);
        assert_ne!(hold, WheelCommand::Velocity(hold.velocity()));
    }
}
