use ozton_math::uom::{
    ConstZero,
    si::f64::{Angle, Length},
};
use ozton_tracking::{mount::Vec3, sampling::TrackingBuilder, sources::vex::DistanceTracker};
use vex_sdk_mock as _;
use vexide::{prelude::Peripherals, runtime::block_on};

#[test]
fn disconnected_distance_sensor_keeps_its_mount_and_allows_tracking_to_start() {
    // The mock SDK reports every smart port as disconnected.
    let peripherals = Peripherals::take().unwrap();
    let range = DistanceTracker::new(
        peripherals.port_7,
        Vec3::new(Length::ZERO, Length::ZERO, Length::ZERO),
        Angle::ZERO,
    );
    let mut tracking = block_on(TrackingBuilder::new().with_dsr([range]).build()).unwrap();

    assert_eq!(tracking.range_mounts().len(), 1);
    assert!(
        tracking
            .capture([None; 2], 0)
            .ranges
            .iter()
            .all(Option::is_none)
    );
}
