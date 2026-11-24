use ozton_math::{
    geometry::compass_heading,
    localization::observation::{
        DisplacementObservation, HeadingObservation, MAX_RANGES, Measurement, ObservationLayout,
        RangeObservation, SensorSample, Snapshot,
    },
    uom::si::f64::Length,
};

use crate::MotionObservations;

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
        MotionObservations,
    ) {
        let mut snapshot = Snapshot {
            time: sample.time,
            ..Snapshot::default()
        };
        self.wheel_travel.fill(None);
        let mut ranges = [None; MAX_RANGES];
        let mut rates = MotionObservations::default();
        let imu_source = sample.observations.iter().find_map(|observation| {
            matches!(observation.measurement, Measurement::Heading(_)).then_some(observation.source)
        });
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
                Measurement::Velocity(velocity) if velocity.is_finite() => {
                    rates.forward_rate = Some(velocity.y);
                }
                Measurement::Rotation(rate) if rate.is_finite() => {
                    if Some(observation.source) == imu_source || rates.yaw_rate.is_none() {
                        rates.yaw_rate = Some(rate.z);
                    }
                }
                Measurement::Acceleration(acceleration) if acceleration.is_finite() => {
                    rates.acceleration = Some(acceleration.truncate());
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
        (snapshot, &self.wheel_travel, ranges, rates)
    }
}
