use ozton_math::{
    geometry::compass_heading,
    localization::observation::{
        DisplacementObservation, HeadingObservation, MAX_RANGES, Measurement, ObservationLayout,
        RangeObservation, SensorSample, Snapshot,
    },
    uom::si::f64::Length,
};

pub(super) struct Observations {
    wheel_sources: Vec<usize>,
    range_sources: Vec<usize>,
    wheel_travel: Vec<Option<Length>>,
}

impl Observations {
    pub(super) fn new(layout: ObservationLayout<'_>) -> Self {
        Self {
            wheel_sources: layout.wheels.iter().map(|&(source, _)| source).collect(),
            range_sources: layout.ranges.iter().map(|&(source, _)| source).collect(),
            wheel_travel: vec![None; layout.wheels.len()],
        }
    }

    pub(super) fn read(
        &mut self,
        sample: SensorSample<'_>,
    ) -> (
        Snapshot,
        &[Option<Length>],
        [Option<RangeObservation>; MAX_RANGES],
    ) {
        let mut snapshot = Snapshot {
            time: sample.time,
            ..Snapshot::default()
        };
        self.wheel_travel.fill(None);
        let mut ranges = [None; MAX_RANGES];
        for observation in sample.observations {
            match observation.measurement {
                Measurement::Displacement(DisplacementObservation::Differential(positions)) => {
                    snapshot.drive = positions;
                }
                Measurement::Displacement(DisplacementObservation::ProjectedWheel(distance)) => {
                    if let Some(index) = self
                        .wheel_sources
                        .iter()
                        .position(|&source| source == observation.source)
                    {
                        self.wheel_travel[index] = Some(distance);
                    }
                }
                Measurement::Heading(heading) if snapshot.imu.is_none() => {
                    let heading = match heading {
                        HeadingObservation::Compass(angle) => compass_heading(angle),
                        HeadingObservation::Counterclockwise(angle) => angle,
                    };
                    snapshot.imu = heading.value.is_finite().then_some(heading);
                }
                Measurement::Range(range) => {
                    if let Some(index) = self
                        .range_sources
                        .iter()
                        .position(|&source| source == observation.source)
                    {
                        ranges[index] = Some(range);
                    }
                }
                _ => {}
            }
        }
        (snapshot, &self.wheel_travel, ranges)
    }
}
