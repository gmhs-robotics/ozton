//! Compile-time source capabilities, combined by heterogeneous source tuples.
use core::marker::PhantomData;

use ozton_math::uom::si::f64::Angle;

use crate::{ReadHeading, ReadTilt, range::{RangeSensorConfig, RangeSource}};

/// Type-level boolean used to combine independent hardware capabilities.
pub trait Flag {
    const PRESENT: bool;
    type Or<Rhs: Flag>: Flag;
}

pub struct No;
pub struct Yes;

impl Flag for No {
    const PRESENT: bool = false;
    type Or<Rhs: Flag> = Rhs;
}

impl Flag for Yes {
    const PRESENT: bool = true;
    type Or<Rhs: Flag> = Yes;
}

/// Bound implemented only for a present capability.
pub trait Available: Flag {}
impl Available for Yes {}

/// Drive, heading, tilt and range capabilities, respectively.
pub struct Capabilities<D = No, H = No, T = No, R = No>(PhantomData<(D, H, T, R)>);

pub type Passive = Capabilities;
pub type Drive = Capabilities<Yes>;
pub type Orientation = Capabilities<No, Yes, Yes>;
pub type Range = Capabilities<No, No, No, Yes>;

#[doc(hidden)]
pub trait HeadingAccess<S>: Flag {
    fn read(source: &S) -> Option<Angle>;
}
impl<S> HeadingAccess<S> for No {
    fn read(_: &S) -> Option<Angle> {
        None
    }
}
impl<S: ReadHeading> HeadingAccess<S> for Yes {
    fn read(source: &S) -> Option<Angle> {
        source.read_heading()
    }
}

#[doc(hidden)]
pub trait TiltAccess<S>: Flag {
    fn read(source: &S) -> Option<[Angle; 2]>;
}
impl<S> TiltAccess<S> for No {
    fn read(_: &S) -> Option<[Angle; 2]> {
        None
    }
}
impl<S: ReadTilt> TiltAccess<S> for Yes {
    fn read(source: &S) -> Option<[Angle; 2]> {
        source.read_tilt()
    }
}

#[doc(hidden)]
pub trait RangeAccess<S>: Flag {
    fn mount(source: &S) -> Option<RangeSensorConfig>;
}
impl<S> RangeAccess<S> for No {
    fn mount(_: &S) -> Option<RangeSensorConfig> {
        None
    }
}
impl<S: RangeSource> RangeAccess<S> for Yes {
    fn mount(source: &S) -> Option<RangeSensorConfig> {
        Some(source.range_config())
    }
}

#[cfg(feature = "vex")]
use crate::odometric::DifferentialTracking;

#[doc(hidden)]
pub trait DriveAccess<S>: Flag {
    #[cfg(feature = "vex")]
    fn get(source: &S) -> Option<&DifferentialTracking>;
    #[cfg(feature = "vex")]
    fn get_mut(source: &mut S) -> Option<&mut DifferentialTracking>;
}
impl<S> DriveAccess<S> for No {
    #[cfg(feature = "vex")]
    fn get(_: &S) -> Option<&DifferentialTracking> {
        None
    }
    #[cfg(feature = "vex")]
    fn get_mut(_: &mut S) -> Option<&mut DifferentialTracking> {
        None
    }
}

#[cfg(feature = "vex")]
impl DriveAccess<DifferentialTracking> for Yes {
    fn get(source: &DifferentialTracking) -> Option<&DifferentialTracking> {
        Some(source)
    }
    fn get_mut(source: &mut DifferentialTracking) -> Option<&mut DifferentialTracking> {
        Some(source)
    }
}

/// Source capability declaration. Each present flag requires the corresponding
/// concrete hardware interface, so a declaration cannot substitute for a reader.
pub trait SourceCapabilities<S> {
    type Drive: Flag;
    type Heading: Flag;
    type Tilt: Flag;
    type Range: Flag;

    fn heading(source: &S) -> Option<Angle>;
    fn tilt(source: &S) -> Option<[Angle; 2]>;
    fn range_mount(source: &S) -> Option<RangeSensorConfig>;

    #[cfg(feature = "vex")]
    fn drive(source: &S) -> Option<&DifferentialTracking>;
    #[cfg(feature = "vex")]
    fn drive_mut(source: &mut S) -> Option<&mut DifferentialTracking>;
}

impl<S, D, H, T, R> SourceCapabilities<S> for Capabilities<D, H, T, R>
where
    D: DriveAccess<S>,
    H: HeadingAccess<S>,
    T: TiltAccess<S>,
    R: RangeAccess<S>,
{
    type Drive = D;
    type Heading = H;
    type Tilt = T;
    type Range = R;

    fn heading(source: &S) -> Option<Angle> {
        H::read(source)
    }
    fn tilt(source: &S) -> Option<[Angle; 2]> {
        T::read(source)
    }
    fn range_mount(source: &S) -> Option<RangeSensorConfig> {
        R::mount(source)
    }

    #[cfg(feature = "vex")]
    fn drive(source: &S) -> Option<&DifferentialTracking> {
        D::get(source)
    }
    #[cfg(feature = "vex")]
    fn drive_mut(source: &mut S) -> Option<&mut DifferentialTracking> {
        D::get_mut(source)
    }
}
