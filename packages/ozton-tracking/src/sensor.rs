use std::{cell::RefCell, rc::Rc, vec::Vec};

use vexide::{
    adi::encoder::AdiEncoder,
    math::Angle,
    smart::{
        PortError,
        gps::GpsSensor,
        imu::{InertialError, InertialSensor},
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

/// A "gyroscope," or a sensor that measures the robot's heading and angular velocity
pub trait Gyro {
    /// The error returned when [`Gyro::heading`] or [`Gyro::angular_velocity`] fails. This
    /// describes a hardware error.
    type Error;

    /// Returns the heading of the robot as an [`Angle`]
    fn heading(&self) -> Result<Angle, Self::Error>;

    /// Returns the horizontal angular velocity
    fn angular_velocity(&self) -> Result<f64, Self::Error>;
}

impl Gyro for InertialSensor {
    type Error = InertialError;

    fn heading(&self) -> Result<Angle, Self::Error> {
        InertialSensor::heading(self).map(|angle| Angle::FULL_TURN - angle)
    }

    fn angular_velocity(&self) -> Result<f64, Self::Error> {
        InertialSensor::gyro_rate(self)
            .map(|rate| (-rate.z).to_radians())
            .map_err(|err| InertialError::Port { source: err })
    }
}

impl Gyro for Rc<InertialSensor> {
    type Error = InertialError;

    fn heading(&self) -> Result<Angle, Self::Error> {
        <InertialSensor as Gyro>::heading(self.as_ref())
    }

    fn angular_velocity(&self) -> Result<f64, Self::Error> {
        <InertialSensor as Gyro>::angular_velocity(self.as_ref())
    }
}

impl Gyro for GpsSensor {
    type Error = PortError;

    fn heading(&self) -> Result<Angle, Self::Error> {
        GpsSensor::heading(self)
            .map(|angle| Angle::from_degrees(-angle.as_degrees()).wrapped_full())
    }

    fn angular_velocity(&self) -> Result<f64, Self::Error> {
        GpsSensor::gyro_rate(self).map(|rate| (-rate.z).to_radians())
    }
}

/// Blanket implementation for all `Rc<RefCell<T>>` wrappers of already implemented sensors.
impl<T: RotarySensor> RotarySensor for Rc<RefCell<T>> {
    type Error = <T as RotarySensor>::Error;

    fn position(&self) -> Result<Angle, Self::Error> {
        self.borrow().position()
    }
}
