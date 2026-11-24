use ozton_math::{
    qty,
    uom::si::f64::{
        Acceleration, AngularAcceleration, AngularVelocity, ElectricPotential, Length, Time,
        Velocity,
    },
};
const SPEED: Velocity = qty!(3 m/s);
const ACCEL: Acceleration = qty!(3 m/s^2);
const TURN_ACCEL: AngularAcceleration = qty!(12 rad/s^2);
const TURN_RATE: AngularVelocity = qty!(180 deg/s);
const DISTANCE: Length = qty!(-2 in);
const VOLTAGE: ElectricPotential = qty!(6 V);
const TIME: Time = qty!(10 ms);
#[test]
fn shorthand_quantities_are_const_and_convert_to_si() {
    assert_eq!(SPEED.value, 3.0);
    assert_eq!(ACCEL.value, 3.0);
    assert_eq!(TURN_ACCEL.value, 12.0);
    assert_eq!(TURN_RATE.value, core::f64::consts::PI);
    assert_eq!(DISTANCE.value, -0.0508);
    assert_eq!(VOLTAGE.value, 6.0);
    assert_eq!(TIME.value, 0.01);
    let speed = 2.0;
    assert_eq!(qty!((speed * 2.0) ft/s).value, 1.2192);
}
