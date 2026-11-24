//! Typed SI quantities and position literals. Shorthand physical units work in constants.
//!
//! ```
//! use ozton_math::units::{Vec3, qty, position_xy, position_xyh, position_xyz, position_xyzh};
//! use ozton_math::{geometry::{DVec2, Pose}, uom::si::f64::{Angle, Length}};
//!
//! const MOUNT: Vec3<Length> = position_xyz!(-2 in, 0.1 m, 25 mm);
//! const AIM: (Vec3<Length>, Angle) = position_xyzh!(-2 in, 0.1 m, 25 mm, 90 deg);
//! const TARGET: DVec2 = position_xy!(2 m, 3 m);
//! const POSE: Pose = position_xyh!(2 m, 3 m, 90 deg);
//! assert_eq!(MOUNT.right, qty!(-2 in));
//! assert_eq!(AIM, (MOUNT, qty!(90 deg)));
//! assert_eq!(POSE.position, TARGET);
//! assert_eq!(POSE.heading, AIM.1);
//! assert_eq!(qty!(2.5 m/s).value, 2.5);
//! ```

use uom::si::f64::Length;

pub use crate::{position_xy, position_xyh, position_xyz, position_xyzh, qty};

/// Three coordinates in right, forward, up order.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec3<T> {
    pub right: T,
    pub forward: T,
    pub up: T,
}

impl<T> Vec3<T> {
    pub const fn new(right: T, forward: T, up: T) -> Self {
        Self { right, forward, up }
    }
}

impl Vec3<Length> {
    /// Returns whether every coordinate is finite.
    pub const fn is_finite(self) -> bool {
        self.right.value.is_finite() && self.forward.value.is_finite() && self.up.value.is_finite()
    }
}

/// Construct a typed physical quantity, using familiar unit notation.
/// All listed shorthand units work in constants. Use parentheses for a computed
/// value, e.g. `qty!((speed * 2.0) m/s)`.
/// For a less common uom unit use `qty!(value, module::Quantity, unit)`
/// (runtime construction).
///
/// ```compile_fail
/// use ozton_math::{qty, uom::si::f64::AngularAcceleration};
/// const BAD: AngularAcceleration = qty!(3 m/s^2);
/// ```
#[macro_export]
macro_rules! qty {
    ($value:expr, $module:ident :: $ty:ident, $unit:ident $(,)?) => {
        $crate::uom::si::f64::$ty::new::<$crate::uom::si::$module::$unit>($value as f64)
    };
    ($value:literal $($unit:tt)+) => { $crate::qty!(@unit $value; $($unit)+) };
    (($value:expr) $($unit:tt)+) => { $crate::qty!(@unit $value; $($unit)+) };
    (@unit $value:expr; m) => { $crate::qty!(@si $value, Length, 1.0) };
    (@unit $value:expr; mm) => { $crate::qty!(@si $value, Length, 0.001) };
    (@unit $value:expr; cm) => { $crate::qty!(@si $value, Length, 0.01) };
    (@unit $value:expr; km) => { $crate::qty!(@si $value, Length, 1000.0) };
    (@unit $value:expr; in) => { $crate::qty!(@si $value, Length, 0.0254) };
    (@unit $value:expr; ft) => { $crate::qty!(@si $value, Length, 0.3048) };
    (@unit $value:expr; yd) => { $crate::qty!(@si $value, Length, 0.9144) };
    (@unit $value:expr; rad) => { $crate::qty!(@si $value, Angle, 1.0) };
    (@unit $value:expr; deg) => { $crate::qty!(@si $value, Angle, ::core::f64::consts::PI / 180.0) };
    (@unit $value:expr; rev) => { $crate::qty!(@si $value, Angle, ::core::f64::consts::TAU) };
    (@unit $value:expr; turn) => { $crate::qty!(@si $value, Angle, ::core::f64::consts::TAU) };
    (@unit $value:expr; m/s) => { $crate::qty!(@si $value, Velocity, 1.0) };
    (@unit $value:expr; cm/s) => { $crate::qty!(@si $value, Velocity, 0.01) };
    (@unit $value:expr; mm/s) => { $crate::qty!(@si $value, Velocity, 0.001) };
    (@unit $value:expr; in/s) => { $crate::qty!(@si $value, Velocity, 0.0254) };
    (@unit $value:expr; ft/s) => { $crate::qty!(@si $value, Velocity, 0.3048) };
    (@unit $value:expr; km/h) => { $crate::qty!(@si $value, Velocity, 1000.0 / 3600.0) };
    (@unit $value:expr; m/s^2) => { $crate::qty!(@si $value, Acceleration, 1.0) };
    (@unit $value:expr; cm/s^2) => { $crate::qty!(@si $value, Acceleration, 0.01) };
    (@unit $value:expr; mm/s^2) => { $crate::qty!(@si $value, Acceleration, 0.001) };
    (@unit $value:expr; in/s^2) => { $crate::qty!(@si $value, Acceleration, 0.0254) };
    (@unit $value:expr; ft/s^2) => { $crate::qty!(@si $value, Acceleration, 0.3048) };
    (@unit $value:expr; rad/s) => { $crate::qty!(@si $value, AngularVelocity, 1.0) };
    (@unit $value:expr; deg/s) => { $crate::qty!(@si $value, AngularVelocity, ::core::f64::consts::PI / 180.0) };
    (@unit $value:expr; rev/s) => { $crate::qty!(@si $value, AngularVelocity, ::core::f64::consts::TAU) };
    (@unit $value:expr; rpm) => { $crate::qty!(@si $value, AngularVelocity, ::core::f64::consts::TAU / 60.0) };
    (@unit $value:expr; rad/s^2) => { $crate::qty!(@si $value, AngularAcceleration, 1.0) };
    (@unit $value:expr; deg/s^2) => { $crate::qty!(@si $value, AngularAcceleration, ::core::f64::consts::PI / 180.0) };
    (@unit $value:expr; rev/s^2) => { $crate::qty!(@si $value, AngularAcceleration, ::core::f64::consts::TAU) };
    (@unit $value:expr; m/s^3) => { $crate::qty!(@si $value, Jerk, 1.0) };
    (@unit $value:expr; rad/s^3) => { $crate::qty!(@si $value, AngularJerk, 1.0) };
    (@unit $value:expr; deg/s^3) => { $crate::qty!(@si $value, AngularJerk, ::core::f64::consts::PI / 180.0) };
    (@unit $value:expr; s) => { $crate::qty!(@si $value, Time, 1.0) };
    (@unit $value:expr; ms) => { $crate::qty!(@si $value, Time, 0.001) };
    (@unit $value:expr; us) => { $crate::qty!(@si $value, Time, 0.000001) };
    (@unit $value:expr; min) => { $crate::qty!(@si $value, Time, 60.0) };
    (@unit $value:expr; h) => { $crate::qty!(@si $value, Time, 3600.0) };
    (@unit $value:expr; V) => { $crate::qty!(@si $value, ElectricPotential, 1.0) };
    (@unit $value:expr; mV) => { $crate::qty!(@si $value, ElectricPotential, 0.001) };
    (@unit $value:expr; A) => { $crate::qty!(@si $value, ElectricCurrent, 1.0) };
    (@unit $value:expr; mA) => { $crate::qty!(@si $value, ElectricCurrent, 0.001) };
    (@unit $value:expr; ohm) => { $crate::qty!(@si $value, ElectricalResistance, 1.0) };
    (@unit $value:expr; kg) => { $crate::qty!(@si $value, Mass, 1.0) };
    (@unit $value:expr; g) => { $crate::qty!(@si $value, Mass, 0.001) };
    (@unit $value:expr; lb) => { $crate::qty!(@si $value, Mass, 0.45359237) };
    (@unit $value:expr; N) => { $crate::qty!(@si $value, Force, 1.0) };
    (@unit $value:expr; N*m) => { $crate::qty!(@si $value, Torque, 1.0) };
    (@unit $value:expr; W) => { $crate::qty!(@si $value, Power, 1.0) };
    (@unit $value:expr; J) => { $crate::qty!(@si $value, Energy, 1.0) };
    (@unit $value:expr; Hz) => { $crate::qty!(@si $value, Frequency, 1.0) };
    (@unit $value:expr; Pa) => { $crate::qty!(@si $value, Pressure, 1.0) };
    (@unit $value:expr; kPa) => { $crate::qty!(@si $value, Pressure, 1000.0) };
    (@unit $value:expr; m^2) => { $crate::qty!(@si $value, Area, 1.0) };
    (@unit $value:expr; in^2) => { $crate::qty!(@si $value, Area, 0.00064516) };
    (@unit $value:expr; m^3) => { $crate::qty!(@si $value, Volume, 1.0) };
    (@unit $value:expr; L) => { $crate::qty!(@si $value, Volume, 0.001) };
    (@unit $value:expr; mL) => { $crate::qty!(@si $value, Volume, 0.000001) };
    (@unit $value:expr; 1/m) => { $crate::qty!(@si $value, Curvature, 1.0) };
    (@si $value:expr, $ty:ident, $scale:expr) => {{
        use $crate::uom::{ConstZero, si::f64::$ty};
        $ty { value: ($value as f64) * ($scale), ..$ty::ZERO }
    }};
    (@unit $value:expr; $($unit:tt)+) => {
        compile_error!(concat!("Unsupported qty! unit: ", stringify!($($unit)+)))
    };
}

/// Creates a planar `DVec2` from two lengths in X, Y order. Its coordinates are in metres.
#[macro_export]
macro_rules! position_xy {
    ($x:literal $x_unit:ident, $y:literal $y_unit:ident $(,)?) => {
        {
            let x: $crate::uom::si::f64::Length = $crate::qty!($x $x_unit);
            let y: $crate::uom::si::f64::Length = $crate::qty!($y $y_unit);
            $crate::geometry::DVec2::new(x.value, y.value)
        }
    };
}

/// Creates a `Vec3<Length>` in X, Y, Z order.
#[macro_export]
macro_rules! position_xyz {
    ($x:literal $x_unit:ident, $y:literal $y_unit:ident, $z:literal $z_unit:ident $(,)?) => {
        {
            let x: $crate::uom::si::f64::Length = $crate::qty!($x $x_unit);
            let y: $crate::uom::si::f64::Length = $crate::qty!($y $y_unit);
            let z: $crate::uom::si::f64::Length = $crate::qty!($z $z_unit);
            $crate::units::Vec3::new(x, y, z)
        }
    };
}

/// Creates a planar `Pose` in X, Y, heading order.
#[macro_export]
macro_rules! position_xyh {
    ($x:literal $x_unit:ident, $y:literal $y_unit:ident, $h:literal $h_unit:ident $(,)?) => {
        $crate::geometry::Pose::new($crate::position_xy!($x $x_unit, $y $y_unit), $crate::qty!($h $h_unit))
    };
}

/// Creates a `(Vec3<Length>, Angle)` in X, Y, Z, heading order.
#[macro_export]
macro_rules! position_xyzh {
    ($x:literal $x_unit:ident, $y:literal $y_unit:ident, $z:literal $z_unit:ident, $h:literal $h_unit:ident $(,)?) => {
        ($crate::position_xyz!($x $x_unit, $y $y_unit, $z $z_unit), $crate::qty!($h $h_unit))
    };
}
