//! Named source roles provide capabilities without exposing numeric indices.
use ozton_math::{
    localization::observation::{HasRange, ReadHeading, ReadTilt},
    uom::si::f64::Angle,
};

use super::{
    Capture, SourceLayout, TrackingBuildError, TrackingSources,
    sources::{HeadingSources, RangeSources, Startup, TiltSources},
};
use crate::{TrackingSource, range::RangeSource};

/// Nonempty collection of concrete ranging sources.
///
/// ```compile_fail
/// use ozton_tracking::Sources;
/// let _ = Sources::new().with_ranges(());
/// ```
pub trait RangeBank: TrackingSources {
    #[doc(hidden)]
    fn describe_ranges(&self, layout: &mut SourceLayout) -> Result<(), TrackingBuildError>;
}

macro_rules! range_banks {
    ($($name:ident:$index:tt),+) => {
        impl<$($name: RangeSource),+> RangeBank for ($($name,)+) {
            fn describe_ranges(&self, layout: &mut SourceLayout) -> Result<(), TrackingBuildError> {
                $(layout.register_range(&self.$index)?;)+
                Ok(())
            }
        }
    };
}
range_banks!(A:0);
range_banks!(A:0, B:1);
range_banks!(A:0, B:1, C:2);
range_banks!(A:0, B:1, C:2, D:3);
range_banks!(A:0, B:1, C:2, D:3, E:4);
range_banks!(A:0, B:1, C:2, D:3, E:4, F:5);
range_banks!(A:0, B:1, C:2, D:3, E:4, F:5, G:6);
range_banks!(A:0, B:1, C:2, D:3, E:4, F:5, G:6, H:7);

macro_rules! range_arrays {
    ($($n:literal),+) => {$(
        impl<T: RangeSource> RangeBank for [T; $n] {
            fn describe_ranges(&self, layout: &mut SourceLayout) -> Result<(), TrackingBuildError> {
                for source in self {
                    layout.register_range(source)?;
                }
                Ok(())
            }
        }
    )+};
}
range_arrays!(1, 2, 3, 4, 5, 6, 7, 8);

/// Empty or populated ranging role. Kept separate from RangeBank so an empty
/// role cannot grant the HasRange capability.
#[doc(hidden)]
pub trait RangeRole: TrackingSources {
    const COUNT: usize;
    fn describe_role(&self, layout: &mut SourceLayout) -> Result<(), TrackingBuildError>;
}

impl RangeRole for () {
    const COUNT: usize = 0;

    fn describe_role(&self, _: &mut SourceLayout) -> Result<(), TrackingBuildError> {
        Ok(())
    }
}

impl<R: RangeBank> RangeRole for R {
    const COUNT: usize = R::SOURCE_COUNT;

    fn describe_role(&self, layout: &mut SourceLayout) -> Result<(), TrackingBuildError> {
        self.describe_ranges(layout)
    }
}

/// Inline, role-based sensor composition. The roles determine the available
/// capabilities; the extra bank accepts any supported source collection.
#[must_use]
pub struct Sources<D = (), H = (), R = (), E = ()> {
    pub(super) drive: D,
    orientation: H,
    ranges: R,
    extra: E,
}

impl Default for Sources {
    fn default() -> Self {
        Self::new()
    }
}

impl Sources {
    pub const fn new() -> Self {
        Self {
            drive: (),
            orientation: (),
            ranges: (),
            extra: (),
        }
    }
}

impl<D, H, R, E> Sources<D, H, R, E> {
    #[cfg(feature = "vex")]
    pub fn with_drive(
        self,
        drive: crate::odometric::DifferentialTracking,
    ) -> Sources<(crate::odometric::DifferentialTracking,), H, R, E> {
        Sources {
            drive: (drive,),
            orientation: self.orientation,
            ranges: self.ranges,
            extra: self.extra,
        }
    }

    pub fn with_orientation<T: TrackingSource>(self, orientation: T) -> Sources<D, (T,), R, E> {
        Sources {
            drive: self.drive,
            orientation: (orientation,),
            ranges: self.ranges,
            extra: self.extra,
        }
    }

    pub fn with_ranges<T: RangeBank>(self, ranges: T) -> Sources<D, H, T, E> {
        Sources {
            drive: self.drive,
            orientation: self.orientation,
            ranges,
            extra: self.extra,
        }
    }

    pub fn with_extra<T: TrackingSources>(self, extra: T) -> Sources<D, H, R, T> {
        Sources {
            drive: self.drive,
            orientation: self.orientation,
            ranges: self.ranges,
            extra,
        }
    }
}

impl<D, H, R, E> Sources<D, H, R, E>
where
    Self: TrackingSources,
{
    /// Validate the source layout and initialize the installed devices.
    pub async fn build(self) -> Result<super::Tracking<Self>, TrackingBuildError> {
        super::Tracking::new(self).await
    }
}

impl<D, H: ReadHeading, R, E> ReadHeading for Sources<D, (H,), R, E> {
    fn read_heading(&self) -> Option<Angle> {
        self.orientation.0.read_heading()
    }
}

impl<D, H: ReadTilt, R, E> ReadTilt for Sources<D, (H,), R, E> {
    fn read_tilt(&self) -> Option<[Angle; 2]> {
        self.orientation.0.read_tilt()
    }
}

impl<D, H, R: RangeBank, E> HasRange for Sources<D, H, R, E> {}

impl<D, H, R: RangeBank, E> RangeSources for Sources<D, H, R, E> {}

impl<D, H: ReadHeading, R, E> HeadingSources for Sources<D, (H,), R, E> {
    fn heading(&self) -> Option<Angle> {
        self.read_heading()
    }
}

impl<D, H: ReadTilt, R, E> TiltSources for Sources<D, (H,), R, E> {
    fn tilt(&self) -> Option<[Angle; 2]> {
        self.read_tilt()
    }
}

/// Internal role for the calibrated drive source.
#[doc(hidden)]
pub trait DriveBank: TrackingSources {
    const SELECTED: bool;

    #[cfg(feature = "vex")]
    fn drive(&self) -> Option<&crate::odometric::DifferentialTracking>;
}

impl DriveBank for () {
    const SELECTED: bool = false;

    #[cfg(feature = "vex")]
    fn drive(&self) -> Option<&crate::odometric::DifferentialTracking> {
        None
    }
}

#[cfg(feature = "vex")]
impl DriveBank for (crate::odometric::DifferentialTracking,) {
    const SELECTED: bool = true;

    fn drive(&self) -> Option<&crate::odometric::DifferentialTracking> {
        Some(&self.0)
    }
}

impl<D: DriveBank, H: TrackingSources, R: RangeRole, E: TrackingSources> TrackingSources
    for Sources<D, H, R, E>
{
    const SOURCE_COUNT: usize =
        D::SOURCE_COUNT + E::SOURCE_COUNT + H::SOURCE_COUNT + R::SOURCE_COUNT;
    const DIFFERENTIAL_COUNT: usize = D::DIFFERENTIAL_COUNT
        + E::DIFFERENTIAL_COUNT
        + H::DIFFERENTIAL_COUNT
        + R::DIFFERENTIAL_COUNT;
    const GUARANTEED_RANGE_COUNT: usize = D::GUARANTEED_RANGE_COUNT
        + E::GUARANTEED_RANGE_COUNT
        + H::GUARANTEED_RANGE_COUNT
        + R::COUNT;
    const HAS_SELECTED_DRIVE: bool = D::SELECTED;

    #[cfg(feature = "vex")]
    fn selected_drive(&self) -> Option<&crate::odometric::DifferentialTracking> {
        self.drive.drive()
    }

    fn describe(&self, layout: &mut SourceLayout) -> Result<(), TrackingBuildError> {
        self.drive.describe(layout)?;
        self.extra.describe(layout)?;
        self.orientation.describe(layout)?;
        self.ranges.describe_role(layout)
    }

    #[cfg(feature = "vex")]
    fn diagnostic_imu(&self) -> Option<&crate::inertial::InertialSensorTracking> {
        self.orientation
            .diagnostic_imu()
            .or_else(|| self.extra.diagnostic_imu())
            .or_else(|| self.ranges.diagnostic_imu())
    }

    fn initialization(&mut self) -> Vec<Startup<'_>> {
        let mut all = self.drive.initialization();
        all.extend(self.extra.initialization());
        all.extend(self.orientation.initialization());
        all.extend(self.ranges.initialization());
        all
    }

    fn capture(&mut self, capture: &mut Capture<'_>) {
        self.drive.capture(capture);
        self.extra.capture(capture);
        self.orientation.capture(capture);
        self.ranges.capture(capture);
    }
}
