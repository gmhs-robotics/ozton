use std::{
    cell::RefCell,
    collections::VecDeque,
    path::PathBuf,
    rc::Rc,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use futures::executor::block_on;
use ozton_derive::RecordedRobot;
use ozton_record::{
    PortError, RecordableRobot,
    frame::{FrameRobot, RecordMode, Recording},
};

#[derive(Debug, Clone, Copy, PartialEq)]
struct AppliedFrame {
    mode: RecordMode,
    value: f64,
}

#[derive(Default)]
struct LoggedActuator {
    applied: Rc<RefCell<Vec<AppliedFrame>>>,
}

#[ozton_record::async_trait(?Send)]
impl ozton_record::RecordField for LoggedActuator {
    type Output = f64;

    async fn finalize_frame_value(&self, frame: &Self::Output) -> Self::Output {
        frame + 0.25
    }

    async fn apply_frame_value(
        &mut self,
        frame: &Self::Output,
        mode: RecordMode,
    ) -> Result<(), PortError> {
        self.applied.borrow_mut().push(AppliedFrame {
            mode,
            value: *frame,
        });
        Ok(())
    }
}

#[derive(RecordedRobot)]
struct ScriptedRobot {
    actuator: LoggedActuator,
    #[record(skip)]
    script: RefCell<VecDeque<f64>>,
}

#[ozton_record::async_trait(?Send)]
impl RecordableRobot for ScriptedRobot {
    const UPDATE_INTERVAL: Duration = Duration::from_millis(5);

    async fn get_new_frame(&self) -> Self::Frame {
        Self::Frame {
            actuator: self.script.borrow_mut().pop_front().unwrap_or_default(),
        }
    }
}

#[test]
fn recording_pipeline_records_finalizes_and_replays_loaded_frames() {
    let applied = Rc::new(RefCell::new(Vec::new()));
    let mut robot = ScriptedRobot {
        actuator: LoggedActuator {
            applied: applied.clone(),
        },
        script: RefCell::new(VecDeque::from([0.5])),
    };

    let live_frame = block_on(robot.get_new_frame());
    assert!((live_frame.actuator - 0.5).abs() < 1e-9);

    block_on(robot.apply_frame(&live_frame, RecordMode::Live)).unwrap();

    let recorded_frame = block_on(robot.finalize_frame(&live_frame));
    assert!((recorded_frame.actuator - 0.75).abs() < 1e-9);

    let mut recording = Recording::default();
    recording.push_timed(Duration::ZERO, recorded_frame.clone());

    let path = temp_recording_path();
    recording.save(&path).unwrap();
    let loaded = Recording::<ScriptedRobotFrame>::load(&path).unwrap();

    block_on(loaded.playback(&mut robot));

    let applied = applied.borrow();
    assert_eq!(
        applied[0],
        AppliedFrame {
            mode: RecordMode::Live,
            value: 0.5,
        }
    );
    assert!(
        applied
            .iter()
            .skip(1)
            .all(|frame| frame.mode == RecordMode::Playback && (frame.value - 0.75).abs() < 1e-9),
        "expected playback applications to use finalized frame, got {:?}",
        *applied
    );

    let _ = std::fs::remove_file(path);
}

fn temp_recording_path() -> PathBuf {
    let timestamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    std::env::temp_dir().join(format!(
        "ozton-recording-pipeline-{timestamp}-{}.route",
        std::process::id()
    ))
}
