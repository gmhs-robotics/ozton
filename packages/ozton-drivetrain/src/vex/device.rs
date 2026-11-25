//! Vexide implementations of the portable actuator and sensor interfaces.

use ozton_math::uom::si::{
    angular_velocity::radian_per_second,
    electric_potential::volt,
    f64::{AngularVelocity, ElectricPotential, Torque},
    torque::newton_meter,
};
use vexide::{
    prelude::Motor,
    smart::PortError,
};

use crate::device::{ReadTorque, ReadVelocity, VoltageOutput};

impl VoltageOutput for Motor {
    type Error = PortError;

    fn max_voltage(&self) -> ElectricPotential {
        ElectricPotential::new::<volt>(Motor::max_voltage(self))
    }

    fn set_voltage(&mut self, voltage: ElectricPotential) -> Result<(), Self::Error> {
        Motor::set_voltage(self, voltage.value)
    }
}

impl ReadVelocity for Motor {
    type Error = PortError;

    fn velocity(&self) -> Result<AngularVelocity, Self::Error> {
        Motor::velocity(self).map(|rpm| {
            AngularVelocity::new::<radian_per_second>(rpm * (core::f64::consts::TAU / 60.0))
        })
    }
}

impl ReadTorque for Motor {
    type Error = PortError;

    fn torque(&self) -> Result<Torque, Self::Error> {
        Motor::torque(self).map(Torque::new::<newton_meter>)
    }
}
