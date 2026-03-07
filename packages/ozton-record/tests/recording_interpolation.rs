use std::time::Duration;

use ozton_record::{
    Interpolate,
    frame::Recording,
    rkyv::{Archive, Deserialize, Serialize},
};

#[derive(Archive, Serialize, Deserialize, Default, Clone, Debug)]
#[rkyv(crate = ::ozton_record::rkyv)]
struct SampleFrame {
    analog: f64,
    digital: bool,
}

impl Interpolate for SampleFrame {
    fn interpolate(from: &Self, to: &Self, amount: f64) -> Self {
        Self {
            analog: Interpolate::interpolate(&from.analog, &to.analog, amount),
            digital: Interpolate::interpolate(&from.digital, &to.digital, amount),
        }
    }
}

#[test]
fn frame_at_interpolates_recording_timeline() {
    let mut recording = Recording::default();
    recording.push_timed(
        Duration::ZERO,
        SampleFrame {
            analog: 0.0,
            digital: false,
        },
    );
    recording.push_timed(
        Duration::from_millis(100),
        SampleFrame {
            analog: 1.0,
            digital: true,
        },
    );

    let sample = recording.frame_at(Duration::from_millis(25)).unwrap();
    assert!((sample.analog - 0.25).abs() < 1e-9);
    assert!(!sample.digital);

    let sample = recording.frame_at(Duration::from_millis(75)).unwrap();
    assert!((sample.analog - 0.75).abs() < 1e-9);
    assert!(sample.digital);
}
