//! Common feedforward (open-loop) controllers.

use core::marker::PhantomData;

use ozton_math::{
    si,
    uom::si::f64::{
        Acceleration as LinearAcceleration, Angle, AngularAcceleration, AngularVelocity,
        ElectricPotential, Velocity as LinearVelocity,
    },
};

use super::Feedforward;
use crate::axis::{Acceleration, AxisDimension, Rotary, Velocity};

/// Desired setpoint of a DC motor.
///
/// Describes the setpoint required to control a motor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorFeedforwardSetpoint<D: AxisDimension = Rotary> {
    /// Desired motor velocity.
    pub velocity: Velocity<D>,

    /// Desired instantaneous motor acceleration.
    pub acceleration: Acceleration<D>,
}

/// Ideal DC motor feedforward controller.
///
/// This is a open-loop velocity controller that computes the voltage to
/// maintain an idealized DC motor at a certain velocity and acceleration.
///
/// The controller is implemented according to the following model:
///
/// `V = Kₛ sign(ω) + Kᵥ ω + Kₐ α`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorFeedforward<D: AxisDimension = Rotary> {
    ks: ElectricPotential,
    kv: f64,
    ka: f64,
    axis: PhantomData<D>,
}

impl<D: AxisDimension> MotorFeedforward<D> {
    /// Creates a feedforward model from static voltage and SI model gains.
    ///
    /// # Parameters
    ///
    /// - `ks` - Feedforward constant for static friction compensation.
    /// - `kv` - Feedforward constant for velocity compensation.
    /// - `ka` - Feedforward constant for acceleration compensation.
    pub const fn new(ks: ElectricPotential, kv: f64, ka: f64) -> Self {
        Self {
            ks,
            kv,
            ka,
            axis: PhantomData,
        }
    }

    /// Get the current feedforward constants as a tuple (`ks`, `kv`, `ka`).
    #[must_use]
    pub const fn constants(&self) -> (ElectricPotential, f64, f64) {
        (self.ks, self.kv, self.ka)
    }

    /// Returns the controller's static friction constant (`ks`).
    #[must_use]
    pub const fn ks(&self) -> ElectricPotential {
        self.ks
    }

    /// Returns the controller's velocity constant (`kv`).
    #[must_use]
    pub const fn kv(&self) -> f64 {
        self.kv
    }

    /// Returns the controller's acceleration constant (`kd`).
    #[must_use]
    pub const fn ka(&self) -> f64 {
        self.ka
    }

    /// Sets the feedforward constants to provided values.
    pub const fn set_constants(&mut self, ks: ElectricPotential, kv: f64, ka: f64) {
        self.ks = ks;
        self.kv = kv;
        self.ka = ka;
    }

    /// Sets the controller's static friction constant (`ks`).
    pub const fn set_ks(&mut self, ks: ElectricPotential) {
        self.ks = ks;
    }

    /// Sets the controller's velocity constant (`kv`).
    pub const fn set_kv(&mut self, kv: f64) {
        self.kv = kv;
    }

    /// Sets the controller's acceleration constant (`ka`).
    pub const fn set_ka(&mut self, ka: f64) {
        self.ka = ka;
    }
}

impl<D: AxisDimension> Feedforward for MotorFeedforward<D> {
    type State = MotorFeedforwardSetpoint<D>;
    type Signal = ElectricPotential;

    fn update(
        &mut self,
        setpoint: MotorFeedforwardSetpoint<D>,
        _dt: core::time::Duration,
    ) -> ElectricPotential {
        let static_voltage = if setpoint.velocity.value == 0.0 {
            0.0
        } else {
            static_voltage(self.ks.value, setpoint.velocity.value)
        };
        si(static_voltage
            + self.kv * setpoint.velocity.value
            + self.ka * setpoint.acceleration.value)
    }
}

/// Desired setpoint of a rotating arm.
///
/// Describes the setpoint required to control a rotating arm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArmFeedforwardSetpoint {
    /// Desired position/angle of the arm.
    pub position: Angle,

    /// Desired velocity of the arm.
    pub velocity: AngularVelocity,

    /// Desired acceleration of the arm.
    pub acceleration: AngularAcceleration,
}

/// Feedforward controller for an ideal DC motor attached to a rotational arm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArmFeedforward {
    ks: ElectricPotential,
    kg: ElectricPotential,
    kv: f64,
    ka: f64,
}

impl ArmFeedforward {
    /// Creates a new arm feedforward controller.
    pub const fn new(ks: ElectricPotential, kg: ElectricPotential, kv: f64, ka: f64) -> Self {
        Self { ks, kg, kv, ka }
    }

    /// Get the current feedforward constants as a tuple (`ks`, `kg`, `kv`, `ka`).
    #[must_use]
    pub const fn constants(&self) -> (ElectricPotential, ElectricPotential, f64, f64) {
        (self.ks, self.kg, self.kv, self.ka)
    }

    /// Returns the controller's static friction constant (`ks`).
    #[must_use]
    pub const fn ks(&self) -> ElectricPotential {
        self.ks
    }

    /// Returns the controller's gravity compensation constant (`kg`).
    #[must_use]
    pub const fn kg(&self) -> ElectricPotential {
        self.kg
    }

    /// Returns the controller's velocity constant (`kv`).
    #[must_use]
    pub const fn kv(&self) -> f64 {
        self.kv
    }

    /// Returns the controller's acceleration constant (`kd`).
    #[must_use]
    pub const fn ka(&self) -> f64 {
        self.ka
    }

    /// Sets the feedforward constants to provided values.
    pub const fn set_constants(
        &mut self,
        ks: ElectricPotential,
        kg: ElectricPotential,
        kv: f64,
        ka: f64,
    ) {
        self.ks = ks;
        self.kg = kg;
        self.kv = kv;
        self.ka = ka;
    }

    /// Sets the controller's static friction constant (`ks`).
    pub const fn set_ks(&mut self, ks: ElectricPotential) {
        self.ks = ks;
    }

    /// Sets the controller's gravity compensation constant (`kg`).
    pub const fn set_kg(&mut self, kg: ElectricPotential) {
        self.kg = kg;
    }

    /// Sets the controller's velocity constant (`kv`).
    pub const fn set_kv(&mut self, kv: f64) {
        self.kv = kv;
    }

    /// Sets the controller's acceleration constant (`ka`).
    pub const fn set_ka(&mut self, ka: f64) {
        self.ka = ka;
    }
}

impl Feedforward for ArmFeedforward {
    type State = ArmFeedforwardSetpoint;
    type Signal = ElectricPotential;
    fn update(
        &mut self,
        setpoint: ArmFeedforwardSetpoint,
        _dt: core::time::Duration,
    ) -> ElectricPotential {
        si(self.kg.value * setpoint.position.value.cos()
            + static_voltage(self.ks.value, setpoint.velocity.value)
            + self.kv * setpoint.velocity.value
            + self.ka * setpoint.acceleration.value)
    }
}

/// Setpoint for [`ElevatorFeedforward`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElevatorFeedforwardSetpoint {
    /// Velocity of the elevator.
    pub velocity: LinearVelocity,

    /// Acceleration of the elevator.
    pub acceleration: LinearAcceleration,
}

/// Feedforward controller for an ideal DC motor attached to an elevator mechanism.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElevatorFeedforward {
    ks: ElectricPotential,
    kg: ElectricPotential,
    kv: f64,
    ka: f64,
}

impl ElevatorFeedforward {
    /// Creates a new elevator feedforward controller.
    pub const fn new(ks: ElectricPotential, kg: ElectricPotential, kv: f64, ka: f64) -> Self {
        Self { ks, kg, kv, ka }
    }

    /// Get the current feedforward constants as a tuple (`ks`, `kg`, `kv`, `ka`).
    #[must_use]
    pub const fn constants(&self) -> (ElectricPotential, ElectricPotential, f64, f64) {
        (self.ks, self.kg, self.kv, self.ka)
    }

    /// Returns the controller's static friction constant (`ks`).
    #[must_use]
    pub const fn ks(&self) -> ElectricPotential {
        self.ks
    }

    /// Returns the controller's gravity compensation constant (`kg`).
    #[must_use]
    pub const fn kg(&self) -> ElectricPotential {
        self.kg
    }

    /// Returns the controller's velocity constant (`kv`).
    #[must_use]
    pub const fn kv(&self) -> f64 {
        self.kv
    }

    /// Returns the controller's acceleration constant (`kd`).
    #[must_use]
    pub const fn ka(&self) -> f64 {
        self.ka
    }

    /// Sets the feedforward constants to provided values.
    pub const fn set_constants(
        &mut self,
        ks: ElectricPotential,
        kg: ElectricPotential,
        kv: f64,
        ka: f64,
    ) {
        self.ks = ks;
        self.kg = kg;
        self.kv = kv;
        self.ka = ka;
    }

    /// Sets the controller's static friction constant (`ks`).
    pub const fn set_ks(&mut self, ks: ElectricPotential) {
        self.ks = ks;
    }

    /// Sets the controller's gravity compensation constant (`kg`).
    pub const fn set_kg(&mut self, kg: ElectricPotential) {
        self.kg = kg;
    }

    /// Sets the controller's velocity constant (`kv`).
    pub const fn set_kv(&mut self, kv: f64) {
        self.kv = kv;
    }

    /// Sets the controller's acceleration constant (`ka`).
    pub const fn set_ka(&mut self, ka: f64) {
        self.ka = ka;
    }
}

impl Feedforward for ElevatorFeedforward {
    type State = ElevatorFeedforwardSetpoint;
    type Signal = ElectricPotential;
    fn update(
        &mut self,
        setpoint: ElevatorFeedforwardSetpoint,
        _dt: core::time::Duration,
    ) -> ElectricPotential {
        si(self.kg.value
            + static_voltage(self.ks.value, setpoint.velocity.value)
            + self.kv * setpoint.velocity.value
            + self.ka * setpoint.acceleration.value)
    }
}

fn static_voltage(gain: f64, velocity: f64) -> f64 {
    if velocity == 0.0 {
        0.0
    } else {
        gain * velocity.signum()
    }
}
