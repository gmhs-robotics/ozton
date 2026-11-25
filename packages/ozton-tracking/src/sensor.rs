//! Angular sensor capabilities and aggregation.

use std::{cell::RefCell, rc::Rc};

use ozton_math::uom::si::f64::Angle;

#[cfg(feature = "vex")]
mod vex;

/// Read accumulated angular position in SI radians.
pub trait ReadPosition {
    type Error;

    fn position(&self) -> Result<Angle, Self::Error>;
}

/// Assign an angular position reference.
pub trait SetPosition: ReadPosition {
    fn set_position(&mut self, position: Angle) -> Result<(), Self::Error>;
}

/// Why a sensor group could not produce a position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PositionGroupError<E> {
    Empty,
    InvalidPosition,
    Device(E),
}

impl<T: ReadPosition> ReadPosition for [T] {
    type Error = PositionGroupError<T::Error>;

    fn position(&self) -> Result<Angle, Self::Error> {
        let mut count = 0.0;
        let mut sum = 0.0;
        let mut failed = PositionGroupError::Empty;
        for sensor in self {
            match sensor.position() {
                Ok(position) if position.value.is_finite() => {
                    sum += position.value;
                    count += 1.0;
                }
                Ok(_) => failed = PositionGroupError::InvalidPosition,
                Err(error) => failed = PositionGroupError::Device(error),
            }
        }
        if count == 0.0 {
            Err(failed)
        } else if !sum.is_finite() {
            Err(PositionGroupError::InvalidPosition)
        } else {
            Ok(ozton_math::qty!((sum / count) rad))
        }
    }
}

impl<T: ReadPosition> ReadPosition for Vec<T> {
    type Error = PositionGroupError<T::Error>;

    fn position(&self) -> Result<Angle, Self::Error> {
        self.as_slice().position()
    }
}

impl<T: ReadPosition, const N: usize> ReadPosition for [T; N] {
    type Error = PositionGroupError<T::Error>;

    fn position(&self) -> Result<Angle, Self::Error> {
        self.as_slice().position()
    }
}

impl<T: ReadPosition> ReadPosition for Rc<RefCell<T>> {
    type Error = T::Error;

    fn position(&self) -> Result<Angle, Self::Error> {
        self.borrow().position()
    }
}

impl<T: SetPosition> SetPosition for Rc<RefCell<T>> {
    fn set_position(&mut self, position: Angle) -> Result<(), Self::Error> {
        self.borrow_mut().set_position(position)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Encoder(f64);

    impl ReadPosition for Encoder {
        type Error = core::convert::Infallible;

        fn position(&self) -> Result<Angle, Self::Error> {
            Ok(ozton_math::qty!((self.0) rad))
        }
    }

    #[test]
    fn position_group_averages_valid_radians() {
        assert_eq!(
            [Encoder(1.0), Encoder(f64::NAN), Encoder(3.0)]
                .position()
                .unwrap()
                .value,
            2.0
        );
        assert_eq!(
            <[Encoder; 0]>::default().position(),
            Err(PositionGroupError::Empty)
        );
    }
}
