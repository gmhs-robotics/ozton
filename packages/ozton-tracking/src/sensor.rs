use std::{cell::RefCell, rc::Rc, vec::Vec};

use vexide::{
    adi::encoder::AdiEncoder,
    math::Angle,
    smart::{
        PortError,
        motor::Motor,
        rotation::RotationSensor,
    },
};

/// A sensor that can measure continuous angular rotation, such as an encoder.
pub trait RotarySensor {
    /// The type of error that the device returns when [`RotarySensor::position`]
    /// fails to return a value.
    type Error;

    /// Reads the angular position of the sensor.
    ///
    /// # Errors
    ///
    /// Returns [`Self::Error`] if the reading failed.
    fn position(&self) -> Result<Angle, Self::Error>;
}

macro_rules! impl_rotary_sensor {
    ( $struct:ident, $method:ident, $err:ty) => {
        impl RotarySensor for $struct {
            type Error = $err;

            fn position(&self) -> Result<Angle, Self::Error> {
                $struct::$method(&self)
            }
        }
    };
}

impl_rotary_sensor!(Motor, position, PortError);
impl_rotary_sensor!(RotationSensor, position, PortError);

impl<const TPR: u32> RotarySensor for AdiEncoder<TPR> {
    type Error = PortError;

    fn position(&self) -> Result<Angle, Self::Error> {
        self.position()
    }
}

/// Why a sensor group could not produce a position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RotaryGroupError<E> {
    Empty,
    InvalidPosition,
    Device(E),
}

impl<T: RotarySensor> RotarySensor for [T] {
    type Error = RotaryGroupError<T::Error>;

    fn position(&self) -> Result<Angle, Self::Error> {
        let mut count = 0.0;
        let mut sum = 0.0;
        let mut failed = RotaryGroupError::Empty;
        for sensor in self {
            match sensor.position() {
                Ok(position) if position.as_radians().is_finite() => {
                    sum += position.as_radians();
                    count += 1.0;
                }
                Ok(_) => failed = RotaryGroupError::InvalidPosition,
                Err(error) => failed = RotaryGroupError::Device(error),
            }
        }
        if count == 0.0 {
            Err(failed)
        } else if !sum.is_finite() {
            Err(RotaryGroupError::InvalidPosition)
        } else {
            Ok(Angle::from_radians(sum / count))
        }
    }
}

impl<T: RotarySensor> RotarySensor for Vec<T> {
    type Error = RotaryGroupError<T::Error>;

    fn position(&self) -> Result<Angle, Self::Error> {
        self.as_slice().position()
    }
}

impl<T: RotarySensor, const N: usize> RotarySensor for [T; N] {
    type Error = RotaryGroupError<T::Error>;

    fn position(&self) -> Result<Angle, Self::Error> {
        self.as_slice().position()
    }
}

/// Blanket implementation for all `Rc<RefCell<T>>` wrappers of already implemented sensors.
impl<T: RotarySensor> RotarySensor for Rc<RefCell<T>> {
    type Error = <T as RotarySensor>::Error;

    fn position(&self) -> Result<Angle, Self::Error> {
        self.borrow().position()
    }
}
