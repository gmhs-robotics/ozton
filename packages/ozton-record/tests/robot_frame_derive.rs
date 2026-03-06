#![allow(dead_code)]

use ozton_derive::RobotFrame;
use ozton_record::FrameType;

struct IncludedField;
struct SkippedField;
struct AnotherIncludedField;

impl FrameType for IncludedField {
    type Output = u16;
}

impl FrameType for SkippedField {
    type Output = bool;
}

impl FrameType for AnotherIncludedField {
    type Output = i8;
}

#[derive(RobotFrame)]
struct TestRobot {
    included: IncludedField,
    #[frame(skip)]
    skipped: SkippedField,
    another: AnotherIncludedField,
}

#[test]
fn frame_skip_excludes_marked_field() {
    let frame = TestRobotFrame {
        included: 7u16,
        another: -3i8,
    };

    let TestRobotFrame { included, another } = frame;
    assert_eq!(included, 7);
    assert_eq!(another, -3);
}

#[test]
fn frame_fields_use_frametype_outputs() {
    let frame = TestRobotFrame {
        included: u16::MAX,
        another: i8::MIN,
    };

    let _: u16 = frame.included;
    let _: i8 = frame.another;
}
