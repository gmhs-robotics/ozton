//! Const-friendly physical quantities stored as one SI scalar.
//!
//! ```
//! use ozton_math::units::Length;
//!
//! const TRACK_WIDTH: Length = Length::inches(12.0);
//! assert!((TRACK_WIDTH.as_meters() - Length::feet(1.0).as_meters()).abs() < 1e-12);
//! ```

macro_rules! quantity {
    ($(#[$doc:meta])* $name:ident, $from_si:ident, $as_si:ident; $($from:ident = $scale:expr),* $(,)?) => {
        $(#[$doc])*
        #[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
        #[repr(transparent)]
        pub struct $name(f64);

        impl $name {
            pub const ZERO: Self = Self(0.0);

            #[inline]
            pub const fn $from_si(value: f64) -> Self {
                Self(value)
            }

            #[inline]
            pub const fn $as_si(self) -> f64 {
                self.0
            }

            #[inline]
            pub const fn is_finite(self) -> bool {
                self.0.is_finite()
            }

            $(
                #[inline]
                pub const fn $from(value: f64) -> Self {
                    Self(value * $scale)
                }
            )*
        }
    };
}

const INCH_IN_METERS: f64 = 0.0254;
const FOOT_IN_METERS: f64 = 0.3048;

quantity!(
    /// Length normalized to meters.
    Length, meters, as_meters;
    millimeters = 0.001, centimeters = 0.01, inches = INCH_IN_METERS, feet = FOOT_IN_METERS,
);

impl Length {
    #[inline]
    pub const fn as_inches(self) -> f64 {
        self.0 / INCH_IN_METERS
    }

    #[inline]
    pub const fn as_feet(self) -> f64 {
        self.0 / FOOT_IN_METERS
    }
}

quantity!(
    /// Linear speed normalized to meters per second.
    Speed, meters_per_second, as_meters_per_second;
    inches_per_second = INCH_IN_METERS, feet_per_second = FOOT_IN_METERS,
);
quantity!(
    /// Linear acceleration normalized to meters per second squared.
    Acceleration, meters_per_second_squared, as_meters_per_second_squared;
    inches_per_second_squared = INCH_IN_METERS, feet_per_second_squared = FOOT_IN_METERS,
);

quantity!(
    /// Angular speed normalized to radians per second.
    AngularSpeed, radians_per_second, as_radians_per_second;
    degrees_per_second = core::f64::consts::PI / 180.0,
    turns_per_second = core::f64::consts::TAU,
);
quantity!(
    /// Electrical current normalized to amperes.
    Current, amperes, as_amperes;
    milliamperes = 0.001,
);
quantity!(
    /// Electrical potential normalized to volts.
    Voltage, volts, as_volts;
    millivolts = 0.001,
);
