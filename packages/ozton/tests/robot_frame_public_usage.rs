#![allow(dead_code)]

use ozton::derive::RobotFrame;
use ozton::record::FrameType;

struct Included;
struct Skipped;

impl FrameType for Included {
    type Output = u8;
}

#[derive(RobotFrame)]
struct Robot {
    included: Included,
    #[frame(skip)]
    skipped: Skipped,
}

#[test]
fn derive_works_via_ozton_without_direct_rkyv_dep() {
    let frame = RobotFrame { included: 42 };
    assert_eq!(frame.included, 42);
}
