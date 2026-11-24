use std::rc::Rc;

use ozton_drivetrain::model::Differential;
use ozton_math::{
    geometry::DVec2,
    uom::{
        ConstZero,
        si::{
            f64::{Angle, Length},
            length::meter,
        },
    },
};
use ozton_tracking::{
    DisplacementObservation, HeadingObservation, Measurement, MeasurementSink, Tracking,
    TrackingSource,
    absolute::PoseObservation,
    mount::Vec3,
    odometric::{DifferentialTracking, WheelGearing},
    range::DistanceTracker,
    shared_motors,
};
use vex_sdk_mock as _;
use vexide::{
    prelude::{Direction, Gearset, Motor, Peripherals},
    runtime::block_on,
};

struct FieldFix;

impl TrackingSource for FieldFix {
    fn sample(&mut self, sink: &mut MeasurementSink<'_>) {
        sink.push(Measurement::AbsolutePose(PoseObservation {
            time: sink.time,
            position: DVec2::new(0.25, 0.5),
            heading: None,
        }));
        sink.push(Measurement::Heading(HeadingObservation::Counterclockwise(
            Angle::ZERO,
        )));
    }
}

#[test]
fn mixed_tracking_sources_keep_order_and_unavailable_range_mounts() {
    // The mock SDK reports every smart port as disconnected.
    let peripherals = Peripherals::take().unwrap();
    let left = shared_motors![Motor::new(
        peripherals.port_1,
        Gearset::Blue,
        Direction::Forward
    )];
    let right = shared_motors![Motor::new(
        peripherals.port_2,
        Gearset::Blue,
        Direction::Forward
    )];
    let model = Differential::from_shared(Rc::clone(&left), Rc::clone(&right));
    let gearing = [WheelGearing::new(1, 1)];
    let encoders = DifferentialTracking::new(
        model.shared_motors(),
        Length::new::<meter>(0.30),
        Length::new::<meter>(0.06),
        &[&gearing, &gearing],
    )
    .unwrap();
    assert_eq!(Rc::strong_count(&left), 3);
    assert_eq!(Rc::strong_count(&right), 3);
    let range = DistanceTracker::new(
        peripherals.port_7,
        Vec3::new(Length::ZERO, Length::ZERO, Length::ZERO),
        Angle::ZERO,
    );
    let mut tracking = block_on(Tracking::new((encoders, range, FieldFix))).unwrap();

    assert_eq!(tracking.observation_layout().ranges.len(), 1);
    assert_eq!(tracking.observation_layout().ranges[0].0, 1);
    let sample = tracking.sample(0);
    assert_eq!(sample.observations.len(), 3);
    assert_eq!(sample.observations[0].source, 0);
    assert_eq!(sample.observations[1].source, 2);
    assert!(matches!(
        sample.observations[0].measurement,
        Measurement::Displacement(DisplacementObservation::Differential([None, None]))
    ));
    assert!(matches!(
        sample.observations[1].measurement,
        Measurement::AbsolutePose(pose) if pose.position == DVec2::new(0.25, 0.5)
    ));
    assert!(matches!(
        sample.observations[2].measurement,
        Measurement::Heading(_)
    ));
}
