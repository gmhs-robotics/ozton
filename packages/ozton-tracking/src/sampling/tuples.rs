//! Static heterogeneous tuple dispatch and capability aggregation.
use ozton_math::uom::si::f64::Angle;

use super::{Capture, SourceLayout, TrackingSources, sources::Startup};
use crate::{
    HasRange, ReadHeading, ReadTilt, TrackingSource,
    capabilities::{Available, Flag, No, SourceCapabilities},
};

// Fold the source types, not runtime sensor indices.
macro_rules! any_capability {
    ($cap:ident;) => { No };
    ($cap:ident; $head:ident $(, $tail:ident)*) => {
        <<$head::Capabilities as SourceCapabilities<$head>>::$cap as Flag>::Or<
            any_capability!($cap; $($tail),*)
        >
    };
}

macro_rules! tracking_tuples {
    ($($name:ident:$index:tt),+) => {
        impl<$($name: TrackingSource),+> TrackingSources for ($($name,)+) {
            const SOURCE_COUNT: usize = [$(stringify!($name)),+].len();

            const HAS_SELECTED_DRIVE: bool =
                <any_capability!(Drive; $($name),+) as Flag>::PRESENT;

            #[cfg(feature = "vex")]
            fn selected_drive(&self) -> Option<&crate::odometric::DifferentialTracking> {
                None$(.or_else(|| $name::Capabilities::drive(&self.$index)))+
            }

            const DIFFERENTIAL_COUNT: usize = 0 $(+ $name::DIFFERENTIAL_ENCODERS as usize)+;
            const GUARANTEED_RANGE_COUNT: usize = 0 $(+ $name::RANGE_SLOTS)+;

            fn describe(&self, layout: &mut SourceLayout) -> Result<(), super::TrackingBuildError> {
                $(layout.register(&self.$index)?;)+
                Ok(())
            }

            #[cfg(feature = "vex")]
            fn diagnostic_imu(&self) -> Option<&crate::inertial::InertialSensorTracking> {
                None$(.or_else(|| self.$index.inertial()))+
            }

            fn initialization(&mut self) -> Vec<Startup<'_>> {
                [$(self.$index.initialize()),+].into_iter().flatten().collect()
            }

            fn capture(&mut self, capture: &mut Capture<'_>) {
                $(capture.sample(&mut self.$index);)+
            }
        }

        impl<$($name: TrackingSource),+> HasRange for ($($name,)+)
        where
            any_capability!(Range; $($name),+): Available,
        {}

        impl<$($name: TrackingSource),+> ReadHeading for ($($name,)+)
        where
            any_capability!(Heading; $($name),+): Available,
        {
            fn read_heading(&self) -> Option<Angle> {
                None$(.or_else(|| $name::Capabilities::heading(&self.$index)))+
            }
        }

        impl<$($name: TrackingSource),+> ReadTilt for ($($name,)+)
        where
            any_capability!(Tilt; $($name),+): Available,
        {
            fn read_tilt(&self) -> Option<[Angle; 2]> {
                None$(.or_else(|| $name::Capabilities::tilt(&self.$index)))+
            }
        }

        #[cfg(feature = "vex")]
        impl<$($name: TrackingSource),+> crate::HasDriveWheels for super::Tracking<($($name,)+)>
        where
            any_capability!(Drive; $($name),+): Available,
        {
            fn calibrated_drive(&self) -> &crate::odometric::DifferentialTracking {
                self.sources.selected_drive().expect("statically installed drive")
            }

            fn calibrated_drive_mut(&mut self) -> &mut crate::odometric::DifferentialTracking {
                $(if let Some(drive) = $name::Capabilities::drive_mut(&mut self.sources.$index) {
                    return drive;
                })+
                unreachable!("statically installed drive")
            }
        }
    };
}

tracking_tuples!(A:0);
tracking_tuples!(A:0, B:1);
tracking_tuples!(A:0, B:1, C:2);
tracking_tuples!(A:0, B:1, C:2, D:3);
tracking_tuples!(A:0, B:1, C:2, D:3, E:4);
tracking_tuples!(A:0, B:1, C:2, D:3, E:4, F:5);
tracking_tuples!(A:0, B:1, C:2, D:3, E:4, F:5, G:6);
tracking_tuples!(A:0, B:1, C:2, D:3, E:4, F:5, G:6, H:7);
tracking_tuples!(A:0, B:1, C:2, D:3, E:4, F:5, G:6, H:7, I:8);
tracking_tuples!(A:0, B:1, C:2, D:3, E:4, F:5, G:6, H:7, I:8, J:9);
tracking_tuples!(A:0, B:1, C:2, D:3, E:4, F:5, G:6, H:7, I:8, J:9, K:10);
tracking_tuples!(A:0, B:1, C:2, D:3, E:4, F:5, G:6, H:7, I:8, J:9, K:10, L:11);

