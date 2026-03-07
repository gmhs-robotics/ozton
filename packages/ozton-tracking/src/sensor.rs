use std::{cell::RefCell, rc::Rc, vec::Vec};

use vexide::{
    adi::encoder::AdiEncoder,
    smart::{
        PortError,
        gps::GpsSensor,
        imu::{InertialError, InertialSensor},
        motor::Motor,
        rotation::RotationSensor,
    },
};
use vexide_devices::math::Angle;

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

impl<T: RotarySensor> RotarySensor for Vec<T> {
    type Error = T::Error;

    fn position(&self) -> Result<Angle, Self::Error> {
        // The total motors to be used in the average later
        let mut total_motors = 0;
        let mut degree_sum = 0.0;
        let mut last_error = None;

        for motor in self {
            degree_sum += match motor.position() {
                Ok(position) => {
                    total_motors += 1;
                    position.as_degrees()
                }
                Err(error) => {
                    // Since this motor isn't being counted in the average, decrement the count
                    last_error = Some(error);
                    continue;
                }
            };
        }

        // Handle a case where no motors were added to the total.
        if total_motors == 0 {
            return if let Some(error) = last_error {
                // Return the error from the last motor that failed.
                Err(error)
            } else {
                // This means there were no motors in the group. We don't want to divide by zero here.
                Ok(Angle::ZERO)
            };
        }

        #[allow(clippy::cast_precision_loss)]
        Ok(Angle::from_degrees(degree_sum / f64::from(total_motors)))
    }
}

// Duplicated because rust is stupid and I can't blanket-impl this for AsMut<[T]>.
impl<const N: usize, T: RotarySensor> RotarySensor for [T; N] {
    type Error = T::Error;

    fn position(&self) -> Result<Angle, Self::Error> {
        // The total motors to be used in the average later
        let mut total_motors = 0;
        let mut degree_sum = 0.0;
        let mut last_error = None;

        for motor in self {
            degree_sum += match motor.position() {
                Ok(position) => {
                    total_motors += 1;
                    position.as_degrees()
                }
                Err(error) => {
                    // Since this motor isn't being counted in the average, decrement the count
                    last_error = Some(error);
                    continue;
                }
            };
        }

        // Handle a case where no motors were added to the total.
        if total_motors == 0 {
            return if let Some(error) = last_error {
                // Return the error from the last motor that failed.
                Err(error)
            } else {
                // This means there were no motors in the group. We don't want to divide by zero here.
                Ok(Angle::ZERO)
            };
        }

        #[allow(clippy::cast_precision_loss)]
        Ok(Angle::from_degrees(degree_sum / f64::from(total_motors)))
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
            .map(|rate| rate.z.to_radians())
            .map_err(|err| InertialError::Port { source: err })
    }
}

impl Gyro for GpsSensor {
    type Error = PortError;

    fn heading(&self) -> Result<Angle, Self::Error> {
        GpsSensor::heading(self)
            .map(|angle| Angle::from_degrees(90.0 - angle.as_degrees()).wrapped_full())
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
