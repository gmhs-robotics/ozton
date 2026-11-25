//! Vexide angular sensor adapter.

use ozton_math::uom::si::f64::Angle;
use vexide::{
    adi::encoder::AdiEncoder,
    math::Angle as VexAngle,
    smart::{PortError, motor::Motor, rotation::RotationSensor},
};

use super::{ReadPosition, SetPosition};

macro_rules! angular_position {
    ($device:ty) => {
        impl ReadPosition for $device {
            type Error = PortError;

            fn position(&self) -> Result<Angle, Self::Error> {
                <$device>::position(self)
                    .map(|position| ozton_math::qty!((position.as_radians()) rad))
            }
        }

        impl SetPosition for $device {
            fn set_position(&mut self, position: Angle) -> Result<(), Self::Error> {
                <$device>::set_position(self, VexAngle::from_radians(position.value))
            }
        }
    };
}

angular_position!(Motor);
angular_position!(RotationSensor);

impl<const TPR: u32> ReadPosition for AdiEncoder<TPR> {
    type Error = PortError;

    fn position(&self) -> Result<Angle, Self::Error> {
        AdiEncoder::position(self).map(|position| ozton_math::qty!((position.as_radians()) rad))
    }
}
