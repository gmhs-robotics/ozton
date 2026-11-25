//! Source collections keep devices inline and dispatch periodic samples through their concrete types.

use std::time::Duration;

use ozton_math::localization::observation::SourceAt;

use crate::{
    MeasurementSink, Observation, SourceInitialization, TrackingSource, range::RangeSensorConfig,
};

/// Sources ordered by tuple position or array index.
pub trait TrackingSources {
    #[doc(hidden)]
    const SOURCE_COUNT: usize;

    #[doc(hidden)]
    fn source(&self, index: usize) -> Option<&dyn TrackingSource>;

    #[doc(hidden)]
    fn source_mut(&mut self, index: usize) -> Option<&mut dyn TrackingSource>;

    #[doc(hidden)]
    fn initialization(&mut self) -> Vec<Startup<'_>>;

    #[doc(hidden)]
    fn capture(&mut self, capture: &mut Capture<'_>);
}

type Startup<'a> = <SourceInitialization<'a> as IntoIterator>::Item;

impl TrackingSources for () {
    const SOURCE_COUNT: usize = 0;

    fn source(&self, _: usize) -> Option<&dyn TrackingSource> {
        None
    }

    fn source_mut(&mut self, _: usize) -> Option<&mut dyn TrackingSource> {
        None
    }

    fn initialization(&mut self) -> Vec<Startup<'_>> {
        Vec::new()
    }

    fn capture(&mut self, _: &mut Capture<'_>) {}
}

impl<S: TrackingSource, const N: usize> TrackingSources for [S; N] {
    const SOURCE_COUNT: usize = N;

    fn source(&self, index: usize) -> Option<&dyn TrackingSource> {
        self.get(index).map(|source| source as _)
    }

    fn source_mut(&mut self, index: usize) -> Option<&mut dyn TrackingSource> {
        self.get_mut(index).map(|source| source as _)
    }

    fn initialization(&mut self) -> Vec<Startup<'_>> {
        self.iter_mut()
            .filter_map(TrackingSource::initialize)
            .collect()
    }

    fn capture(&mut self, capture: &mut Capture<'_>) {
        for (index, source) in self.iter_mut().enumerate() {
            capture.sample(index, source);
        }
    }
}

// Generate one non-overlapping implementation for every tuple element.
macro_rules! source_indices {
    (($($all:ident),+);) => {};
    (($($all:ident),+); $name:ident:$index:tt $(, $rest:ident:$other:tt)*) => {
        impl<DriveSelection, $($all),+> SourceAt<$index> for super::Tracking<($($all,)+), DriveSelection> {
            type Source = $name;

            fn source_at(&self) -> &Self::Source {
                &self.sources.$index
            }

            fn source_at_mut(&mut self) -> &mut Self::Source {
                &mut self.sources.$index
            }
        }
        source_indices!(($($all),+); $($rest:$other),*);
    };
}

macro_rules! tracking_tuples {
    ($($name:ident:$index:tt),+) => {
        source_indices!(($($name),+); $($name:$index),+);
        impl<$($name: TrackingSource),+> TrackingSources for ($($name,)+) {
            const SOURCE_COUNT: usize = [$(stringify!($name)),+].len();

            fn source(&self, index: usize) -> Option<&dyn TrackingSource> {
                match index {
                    $($index => Some(&self.$index),)+
                    _ => None,
                }
            }

            fn source_mut(&mut self, index: usize) -> Option<&mut dyn TrackingSource> {
                match index {
                    $($index => Some(&mut self.$index),)+
                    _ => None,
                }
            }

            fn initialization(&mut self) -> Vec<Startup<'_>> {
                [$(self.$index.initialize()),+].into_iter().flatten().collect()
            }

            fn capture(&mut self, capture: &mut Capture<'_>) {
                $(capture.sample($index, &mut self.$index);)+
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

#[doc(hidden)]
pub struct Capture<'a> {
    time: Duration,
    occluded: u8,
    next_range: usize,
    range_layout: &'a [(usize, RangeSensorConfig)],
    observations: &'a mut Vec<Observation>,
}

impl<'a> Capture<'a> {
    pub(super) fn new(
        time: Duration,
        occluded: u8,
        range_layout: &'a [(usize, RangeSensorConfig)],
        observations: &'a mut Vec<Observation>,
    ) -> Self {
        Self {
            time,
            occluded,
            next_range: 0,
            range_layout,
            observations,
        }
    }

    fn sample(&mut self, index: usize, source: &mut impl TrackingSource) {
        let range_index = self
            .range_layout
            .get(self.next_range)
            .is_some_and(|(source, _)| *source == index)
            .then(|| {
                let range = self.next_range;
                self.next_range += 1;
                range
            });
        source.sample(&mut MeasurementSink {
            time: self.time,
            source: index,
            range_index,
            range_occluded: range_index.is_some_and(|range| self.occluded & (1 << range) != 0),
            observations: self.observations,
        });
    }
}
