use ozton_math::{geometry::compass_heading, uom::si::f64::Length};
use ozton_tracking::{
    DisplacementObservation, HeadingObservation, Measurement, Tracking,
    range::{MAX_RANGES, RangeObservation},
    sampling::SensorSample,
    snapshot::Snapshot,
};

pub(super) struct Observations {
    wheel_sources: Vec<usize>,
    range_sources: Vec<usize>,
    wheel_travel: Vec<Option<Length>>,
}

impl Observations {
    pub(super) fn new(tracking: &Tracking) -> Self {
        Self {
            wheel_sources: tracking
                .wheel_mounts()
                .iter()
                .map(|&(source, _)| source)
                .collect(),
            range_sources: tracking.range_sources().to_vec(),
            wheel_travel: vec![None; tracking.wheel_mounts().len()],
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
