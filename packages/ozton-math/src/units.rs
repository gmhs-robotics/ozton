//! Ozton uses `uom` quantities in SI base units. The helpers can be used in
//! constants where `Quantity::new::<Unit>` is unavailable.
//!
//! ```
//! use ozton_math::units::{Length, inches, uom::si::length::foot};
//!
//! const TRACK_WIDTH: Length = inches(12.0);
//! assert!((TRACK_WIDTH.get::<foot>() - 1.0).abs() < 1e-12);
//! ```

use core::marker::PhantomData;

use uom::si::{Dimension, Quantity, SI};
pub use uom::{
    self,
    si::f64::{
        Acceleration, AngularVelocity, ElectricCurrent, ElectricPotential, Length, Velocity,
    },
};

const fn si<D: Dimension + ?Sized>(value: f64) -> Quantity<D, SI<f64>, f64> {
    Quantity {
        dimension: PhantomData,
        units: PhantomData,
        value,
    }
}

macro_rules! constructors {
    ($($name:ident -> $quantity:ident $(, $scale:expr)?);* $(;)?) => {
        $(
            #[inline]
            pub const fn $name(value: f64) -> $quantity {
                si(value $(* $scale)?)
            }
        )*
    };
}

constructors! {
    meters -> Length;
    millimeters -> Length, 0.001;
    centimeters -> Length, 0.01;
    inches -> Length, 0.0254;
    feet -> Length, 0.3048;
    meters_per_second -> Velocity;
    meters_per_second_squared -> Acceleration;
    radians_per_second -> AngularVelocity;
    amperes -> ElectricCurrent;
    volts -> ElectricPotential;
}
