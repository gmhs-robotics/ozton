//! SI dimensions for linear and rotary control models.
use crate::uom::si::{self, Dimension, Quantity, SI};

mod sealed {
    pub trait Sealed {}
}

/// Selects the position, velocity and acceleration dimensions of a control axis.
pub trait AxisDimension: sealed::Sealed + Copy {
    type Position: Dimension + ?Sized;
    type Velocity: Dimension + ?Sized;
    type Acceleration: Dimension + ?Sized;
}

/// A mechanism with angular travel.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Rotary;

/// A mechanism with linear travel.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Linear;

impl sealed::Sealed for Rotary {}
impl sealed::Sealed for Linear {}

impl AxisDimension for Rotary {
    type Position = si::angle::Dimension;
    type Velocity = si::angular_velocity::Dimension;
    type Acceleration = si::angular_acceleration::Dimension;
}

impl AxisDimension for Linear {
    type Position = si::length::Dimension;
    type Velocity = si::velocity::Dimension;
    type Acceleration = si::acceleration::Dimension;
}

pub type Position<D> = Quantity<<D as AxisDimension>::Position, SI<f64>, f64>;
pub type Velocity<D> = Quantity<<D as AxisDimension>::Velocity, SI<f64>, f64>;
pub type Acceleration<D> = Quantity<<D as AxisDimension>::Acceleration, SI<f64>, f64>;
